// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Map.Entry;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.vision.CustomEstimator.PoseStrategy;
import frc.robot.util.MoreMath;
import frc.robot.util.vision.CustomEstimate;
import frc.robot.util.vision.CustomEstimator;
import frc.robot.util.vision.PoseBufferWrapper;

public class Vision extends SubsystemBase {
  public Swerve swerve;

  public List<CustomEstimator> estimators;
  private Field2d field = new Field2d();

  public List<List<PhotonTrackedTarget>> targetsPerCam;
  public List<Optional<CustomEstimate>> estimates;

  private AprilTagFieldLayout atfl;

  private FuseVisionEstimate defaultCommand;

  public Vision(Swerve swerve) {
    this.swerve = swerve;

    initAtfl();
    initPoseEstimators();
    initEstimates();
    initBuffer();
    initField();

    defaultCommand = new FuseVisionEstimate(this, ConfidenceStrat.ONLY_COMMUNITY_AND_FEEDER);
    setDefaultCommand(defaultCommand);
  }

  private void initBuffer() {
    PoseBufferWrapper.createBuffers(swerve::getPose, swerve::getWheelPose, swerve::getModulePositions,
        swerve::getCtsGyroRotWithOffset, swerve.kinematics, swerve::setResetBuffer);
  }

  public void setConfidenceStrat(ConfidenceStrat confidenceStrat) {
    defaultCommand.setConfidenceStrat(confidenceStrat);
  }

  public void periodic() {
    updateAtflOrigin();
    updateEstimates();
    logTagPoses();
  }

  /* I hate null pointer exceptions */
  public void initEstimates() {
    this.targetsPerCam = new ArrayList<>(Collections.nCopies(estimators.size(), new ArrayList<PhotonTrackedTarget>()));
    this.estimates = new ArrayList<>(Collections.nCopies(estimators.size(), Optional.empty()));
  }

  private void initField() {
    SmartDashboard.putData("vision/vision field", field);
    field.setRobotPose(new Pose2d(-100, -100, new Rotation2d()));
  }

  /**
   * 
   */
  public void updateAtflOrigin() {
    // Set AprilTagFieldLayout Alliance Color
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      this.atfl.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } else {
      this.atfl.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    }
  }

  public AprilTagFieldLayout getAtfl() {
    return atfl;
  }

  public void updateEstimates() {
    estimates.clear();
    targetsPerCam.clear();

    /* For some reason cam in PhotonPoseEstimator is not visible */
    for (var i = 0; i < estimators.size(); i++) {
      var poseEstimator = estimators.get(i);
      var cam = (PhotonCamera) Constants.VISION.CAMS_AND_TRANS.keySet().toArray()[i];

      var targets = cam.getLatestResult().getTargets();
      targetsPerCam.add(targets);

      var estimate = poseEstimator.update();
      estimates.add(estimate);
    }
  }

  public static boolean isEstimatePresent(Optional<CustomEstimate> estimate) {
    return estimate.isPresent() && estimate.get().best != null && estimate.get().best.getRotation() != null;
  }

  private void initAtfl() {
    try {
      this.atfl = Constants.VISION.ATFL;
      // this.atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e) {//I really wish we could be doing this in constants.java, not here lol (if you can fix this, plz do)
      DriverStation.reportError("Welp, if ur reading this, imma guess ur getting a nullpointer exception.", true);
    }
  }

  private void initPoseEstimators() {
    estimators = new ArrayList<>();
    for (var camAndTransform : Constants.VISION.CAMS_AND_TRANS.entrySet()) {
      estimators.add(
          new CustomEstimator(atfl, PoseStrategy.MIN_AMBIGUITY, camAndTransform.getKey(), camAndTransform.getValue()));
    }
  }

  public void logTagPoses() {
    /* Loop through estimators/cams */
    for (var i = 0; i < estimators.size(); i++) {
      //create camera and transform variables to ease access
      var camAndTrans = (Entry<PhotonCamera, Transform3d>) Constants.VISION.CAMS_AND_TRANS.entrySet().toArray()[i];
      var cam = camAndTrans.getKey();
      var robot2Cam = camAndTrans.getValue();

      //prepare arrays
      var tagIds = new ArrayList<Integer>();
      var tagCorners = new ArrayList<TargetCorner>();
      var fieldToTags = new ArrayList<Pose3d>();
      var fieldToRobots = new ArrayList<Pose3d>();
      var fieldToCams = new ArrayList<Pose3d>();
      var camToTags = new ArrayList<Transform3d>();

      /* Loop through targets */
      for (var target : cam.getLatestResult().getTargets()) {
        var id = target.getFiducialId();
        tagIds.add(id);

        var tagPoseOpt = atfl.getTagPose(id);
        if (tagPoseOpt.isEmpty()) {
          fieldToTags.add(new Pose3d());
          fieldToRobots.add(new Pose3d());
          camToTags.add(new Transform3d());
          tagCorners.addAll(Collections.emptyList());
          continue;
        }

        var fieldToTag = tagPoseOpt.get();
        var fieldToRobot =
            fieldToTag.transformBy(target.getBestCameraToTarget().inverse()).transformBy(robot2Cam.inverse());
        var fieldToCam = fieldToTag.transformBy(target.getBestCameraToTarget().inverse());

        var corners = target.getDetectedCorners(); // actually 4, but who cares about understandable variable naming lol

        fieldToCams.add(fieldToCam);
        fieldToTags.add(fieldToTag);
        fieldToRobots.add(fieldToRobot);
        tagCorners.addAll(corners);
        camToTags.add(target.getBestCameraToTarget());
      }

      /* Prepare values for logging */
      var tagIdsArray = tagIds.stream().mapToLong(Long::valueOf).toArray();

      var fieldToRobotStrs = fieldToRobots.stream().map((estRobotPose) -> MoreMath.pose3dToString(estRobotPose, 1))
          .toList().toArray(new String[0]);

      var fieldToCamStrs = fieldToCams.stream().map((estRobotPose) -> MoreMath.pose3dToString(estRobotPose, 3)).toList()
          .toArray(new String[0]);

      var fieldToTagStrs = fieldToTags.stream().map((tagPose) -> (MoreMath.pose2dToString(tagPose.toPose2d(), 1)))
          .toList().toArray(new String[0]);

      var camToTagStrs = camToTags.stream().map((camToTag) -> (MoreMath.transform3dToString(camToTag, 1))).toList()
          .toArray(new String[0]);

      var tagCornersStr =
          tagCorners.stream().map((corner) -> MoreMath.cornerToString(corner, 1)).toList().toArray(new String[0]);

      var optEstimate = estimates.get(i);
      var estimateStatus = "invalid";
      var estimateStr = "";
      var estimate3dStr = "";
      if (isEstimatePresent(optEstimate)) {
        estimateStatus = "valid";
        var estimate = optEstimate.get();
        var pose = estimate.best.toPose2d();
        this.field.getObject(cam.getName()).setPose(pose);
        estimateStr = MoreMath.pose2dToString(pose, 1);
        estimate3dStr = MoreMath.pose3dToString(estimate.best, 1);
      }

      var timestamp = cam.getLatestResult().getTimestampSeconds();

      var camMatrixPresent = "invalid";
      if (cam.getCameraMatrix().isPresent()) {
        camMatrixPresent = "valid";
      }

      var distCoeffPresent = "invalid";
      if (cam.getDistCoeffs().isPresent()) {
        distCoeffPresent = "valid";
      }

      var camPresent = cam.isConnected() ? "connected" : "disconnected";

      // advantage kit logging (because why not)
      List<Pose3d> akTagPoses = new ArrayList<>();
      List<Integer> akTagIds = new ArrayList<>();
      for (var target : cam.getLatestResult().getTargets()) {
        var swervePose = new Pose3d(swerve.getPose());
        akTagPoses.add(swervePose.transformBy(target.getBestCameraToTarget()));
        akTagIds.add(target.getFiducialId());
      }
      var akTagPosesArray = fieldToTags.toArray(new Pose3d[fieldToTags.size()]);
      var akFieldToCams = fieldToCams.toArray(new Pose3d[fieldToCams.size()]);
      var akFieldToRobots = fieldToRobots.toArray(new Pose3d[fieldToRobots.size()]);

      /* May be slightly cursed */
      var akTagIdsArray = tagIds.stream().mapToLong(Long::valueOf).toArray();

      var path = "vision/" + cam.getName() + "/";
      Logger.getInstance().recordOutput(path + "cam status", camPresent);
      Logger.getInstance().recordOutput(path + "estimate status", estimateStatus);
      if (!estimateStatus.equals("invalid")) {
        Logger.getInstance().recordOutput(path + "estimate", estimateStr);
        Logger.getInstance().recordOutput(path + "3d estimate", estimate3dStr);
      }
      Logger.getInstance().recordOutput(path + "timestamp", timestamp);
      Logger.getInstance().recordOutput(path + "cam matrix", camMatrixPresent);
      Logger.getInstance().recordOutput(path + "dist coeff", distCoeffPresent);

      var targetPath = path + "targets/";
      Logger.getInstance().recordOutput(targetPath + "tag ids", tagIdsArray);
      Logger.getInstance().recordOutput(targetPath + "tag corners", tagCornersStr);
      Logger.getInstance().recordOutput(targetPath + "field to tags", fieldToTagStrs);
      Logger.getInstance().recordOutput(targetPath + "field to cams", fieldToCamStrs);
      Logger.getInstance().recordOutput(targetPath + "field to robots", fieldToRobotStrs);
      Logger.getInstance().recordOutput(targetPath + "cam to tags", camToTagStrs);

      var akPath = path + "ak/";
      Logger.getInstance().recordOutput(akPath + "tag poses", akTagPosesArray);
      Logger.getInstance().recordOutput(akPath + "tag ids", akTagIdsArray);
      Logger.getInstance().recordOutput(akPath + "field to cams", akFieldToCams);
      Logger.getInstance().recordOutput(akPath + "field to robots", akFieldToRobots);


      field.getObject("Robot Pose").setPose(swerve.getPose());
      field.getObject("Wheel Pose").setPose(swerve.getWheelPose());
    }
  }
}
