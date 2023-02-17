// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

public class Vision extends SubsystemBase {
  public PhotonCamera targetCam;
  private PhotonPoseEstimator photonPoseEstimator;

  LinearFilter xFilter = LinearFilter.movingAverage(10);
  LinearFilter yFilter = LinearFilter.movingAverage(10);
  LinearFilter rotFilter = LinearFilter.movingAverage(10);

  public enum ShouldSetVisionPose {
    UPDATE(true), DONT_UPDATE(false);

    public final boolean value;

    ShouldSetVisionPose(boolean value) {
      this.value = value;
    }
  }

  private ShouldSetVisionPose shouldSetVisionPose;
  private Swerve swerve;

  public Vision(Swerve swerve) {
    this(swerve, ShouldSetVisionPose.UPDATE);
  }

  public Vision(Swerve swerve, ShouldSetVisionPose shouldSetVisionPose) {
    this.shouldSetVisionPose = shouldSetVisionPose;
    this.swerve = swerve;
    this.targetCam = new PhotonCamera("target");
    initializePoseEstimator();

    SmartDashboard.putBoolean("Vision/2nd Level Vision Filtering", false);
    SmartDashboard.putBoolean("Vision/Vary Vision StDev By Chassis Spd", true);
  }

  public void periodic() {
    // Set AprilTagFieldLayout Alliance Color
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      this.photonPoseEstimator.getFieldTags().setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } else {
      this.photonPoseEstimator.getFieldTags().setOrigin(OriginPosition.kRedAllianceWallRightSide);
    }

    // Update Vision Estimate
    if (this.shouldSetVisionPose.value && isPoseEstimateValid()) {
      var swervePose = swerve.getPose();
      var swervePose3d = new Pose3d(swervePose.getX(), swervePose.getY(), 0,
          new Rotation3d(0, 0, swervePose.getRotation().getRadians()));
      Optional<EstimatedRobotPose> visionResult = getVisionPose(swervePose);
      Pose2d estimatedRobotPos = visionResult.get().estimatedPose.toPose2d();
      estimatedRobotPos = averageVisionPose(estimatedRobotPos);
      double timestamp = visionResult.get().timestampSeconds;

      /* Log all tag poses */
      List<Pose3d> tagPoses = new ArrayList<>();
      List<Integer> tagIds = new ArrayList<>();
      for (var target : targetCam.getLatestResult().getTargets()) {
        tagPoses.add(swervePose3d.transformBy(target.getBestCameraToTarget()));
        tagIds.add(target.getFiducialId());
      }
      var tagPosesArray = tagPoses.toArray(new Pose3d[tagPoses.size()]);
      /* May be slightly cursed */
      var tagIdsArray = tagIds.stream().mapToLong(Long::valueOf).toArray();

      Logger.getInstance().recordOutput("Vision/TagPoses", tagPosesArray);
      Logger.getInstance().recordOutput("Vision/TagIds", tagIdsArray);

      //Vary St Devs based on Chassis Spd
      double visionStDev = 0.9;
      boolean shouldVaryStDev = SmartDashboard.getBoolean("Vision/Vary Vision StDev By Chassis Spd", true);
      if (shouldVaryStDev) {
        ChassisSpeeds vels = this.swerve.getChassisSpeeds();
        double spds = Math.hypot(vels.vxMetersPerSecond, vels.vyMetersPerSecond);
        if (spds > 1.5) {
          visionStDev = 3;
        }
      }
      Matrix<N3, N1> stDevs = VecBuilder.fill(visionStDev, visionStDev, visionStDev);
      this.swerve.setVisionPose(estimatedRobotPos, timestamp, stDevs);

      SmartDashboard.putBoolean("Vision/Is Updating", true);
    } else {
      averageVisionPose(this.swerve.getPose());
      SmartDashboard.putBoolean("Vision/Is Updating", false);
    }
  }

  public void setShouldSetVisionPose(ShouldSetVisionPose shouldSetVisionPose) {
    this.shouldSetVisionPose = shouldSetVisionPose;
  }

  public AprilTagFieldLayout getAtfl() {
    return photonPoseEstimator.getFieldTags();
  }

  boolean isPoseEstimateValid() {
    boolean isConnected = targetCam.isConnected();
    boolean hasTargets = targetCam.getLatestResult().hasTargets();

    Optional<EstimatedRobotPose> visionResult = getVisionPose(this.swerve.getPose());
    boolean isEstimatePresent = Vision.isVisionPosePresent(visionResult);

    PhotonTrackedTarget best_target = targetCam.getLatestResult().getBestTarget();
    boolean isBestTargetPresent = best_target != null;

    // first set of conditions to satisfy
    if (isConnected && hasTargets && isEstimatePresent && isBestTargetPresent) {
      double ambiguity = best_target.getPoseAmbiguity();
      double deltaTime = Timer.getFPGATimestamp() - visionResult.get().timestampSeconds;
      double distance =
          best_target.getBestCameraToTarget().getTranslation().toTranslation2d().getDistance(new Translation2d());

      boolean shouldFilter = SmartDashboard.getBoolean("Vision/2nd Level Vision Filtering", false);

      // second set of conditions to satisfy
      if ((ambiguity < 5e-2
          // && distance < 2
          && deltaTime < 0.5) || !shouldFilter) {
        return true;
      }
    }
    return false;
  }

  public static boolean isVisionPosePresent(Optional<EstimatedRobotPose> estimatedRobotPos) {
    return estimatedRobotPos.isPresent() && estimatedRobotPos.get().estimatedPose != null
        && estimatedRobotPos.get().estimatedPose.getRotation() != null;
  }

  public Optional<EstimatedRobotPose> getVisionPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public Pose2d averageVisionPose(Pose2d currentEstimate) {
    double x = xFilter.calculate(currentEstimate.getX());
    double y = yFilter.calculate(currentEstimate.getY());

    return new Pose2d(x, y, currentEstimate.getRotation());
  }

  private void initializePoseEstimator() {
    //Create camList
    Transform3d robotToCam = Constants.VISION.ROBOT_TO_CAM;
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<>(targetCam, robotToCam));

    AprilTagFieldLayout atfl = null;
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e) {//I really wish we could be doing this in constants.java, not here lol (if you can fix this, plz do)
      DriverStation.reportError("Welp, if ur reading this, imma guess ur getting a nullpointer exception.", true);
    }
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      atfl.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    }
    this.photonPoseEstimator =
        new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP, targetCam, Constants.VISION.ROBOT_TO_CAM);
  }
}
