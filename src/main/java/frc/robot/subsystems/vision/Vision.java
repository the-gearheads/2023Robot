// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  public PhotonCamera targetCam;
  private PhotonPoseEstimator photonPoseEstimator;

  LinearFilter xFilter = LinearFilter.movingAverage(5);
  LinearFilter yFilter = LinearFilter.movingAverage(5);
  LinearFilter rotFilter = LinearFilter.movingAverage(5);
  private BiConsumer<Pose2d, Double> setVisionPose;
  private Supplier<Pose2d> getPose;

  public enum ShouldSetVisionPose{
    UPDATE(true),DONT_UPDATE(false);
    public final boolean value;

    ShouldSetVisionPose(boolean value) {
      this.value = value;
    }
  }
  private ShouldSetVisionPose shouldSetVisionPose;

  public Vision(Supplier<Pose2d> getPose, BiConsumer<Pose2d, Double> setVisionPose) {
    this(getPose, setVisionPose, ShouldSetVisionPose.UPDATE);
  }

  public Vision(Supplier<Pose2d> getPose, BiConsumer<Pose2d, Double> setVisionPose, ShouldSetVisionPose shouldSetVisionPose) {
    this.shouldSetVisionPose = shouldSetVisionPose;
    this.getPose = getPose;
    this.setVisionPose=setVisionPose;
    this.targetCam = new PhotonCamera("target");
    initializePoseEstimator();
  }

  public void periodic(){
    if(this.shouldSetVisionPose.value && isPoseEstimateValid()){
      Optional<EstimatedRobotPose> visionResult = getVisionPose(this.getPose.get());
      Pose2d estimatedRobotPos = visionResult.get().estimatedPose.toPose2d();
      estimatedRobotPos = averageVisionPose(estimatedRobotPos);
      double timestamp = visionResult.get().timestampSeconds;
      this.setVisionPose.accept(estimatedRobotPos, timestamp);
    }
  }

  public void setShouldSetVisionPose(ShouldSetVisionPose shouldSetVisionPose){
    this.shouldSetVisionPose = shouldSetVisionPose; 
  }

  public AprilTagFieldLayout getAtfl() {
    return photonPoseEstimator.getFieldTags();
  }
  boolean isPoseEstimateValid(){
    boolean isConnected = targetCam.isConnected();
    boolean hasTargets = targetCam.getLatestResult().hasTargets();

    Optional<EstimatedRobotPose> visionResult = getVisionPose(this.getPose.get());
    boolean isEstimatePresent = Vision.isVisionPosePresent(visionResult);
    
    PhotonTrackedTarget best_target = targetCam.getLatestResult().getBestTarget();
    boolean isBestTargetPresent = best_target != null;

    if (isConnected && hasTargets && isEstimatePresent && isBestTargetPresent) {
      double ambiguity = best_target.getPoseAmbiguity();
      double deltaTime = Timer.getFPGATimestamp() - visionResult.get().timestampSeconds;
      double distance = 
      best_target.getBestCameraToTarget()
      .getTranslation().toTranslation2d().getDistance(new Translation2d());

      if(ambiguity < 5e-2 && distance < 2 && deltaTime < 0.5){
        SmartDashboard.putBoolean("Vision/Is Updating", true);
        return false;
      }
    }
    SmartDashboard.putBoolean("Vision/Is Updating", false);
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

  public Pose2d averageVisionPose(Pose2d currentEstimate){
    double x = xFilter.calculate(currentEstimate.getX());
    double y = yFilter.calculate(currentEstimate.getY());

    return new Pose2d(x,y, currentEstimate.getRotation());
  }

  private void initializePoseEstimator() {
    //Create camList
    Transform3d robotToCam = Constants.Vision.robotToCam;
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<>(targetCam, robotToCam));

    AprilTagFieldLayout atfl = null;
    try {
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e) {//I really wish we could be doing this in constants.java, not here lol (if you can fix this, plz do)
      DriverStation.reportError("Welp, if ur reading this, imma guess ur getting a nullpointer exception.", true);
    }
    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      atfl.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    }
    this.photonPoseEstimator =
        new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, targetCam, Constants.Vision.robotToCam);
  }
}
