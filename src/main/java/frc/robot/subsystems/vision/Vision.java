// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Map.Entry;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.AdditionalMathUtils;
import frc.robot.util.multicamvision.ClosestEstimateFinder;
import frc.robot.util.multicamvision.PoseBufferWrapper;

public class Vision extends SubsystemBase {
  public Swerve swerve;

  public ClosestEstimateFinder closestEstimateFinder;
  public List<PhotonPoseEstimator> estimators;
  private Field2d field = new Field2d();

  public List<List<PhotonTrackedTarget>> targetsPerCam;
  public List<Optional<EstimatedRobotPose>> estimates;

  private AprilTagFieldLayout atfl;

  public Vision(Swerve swerve) {
    this.swerve = swerve;

    initAtfl();
    initPoseEstimators();
    initEstimates();
    initClosestEstimateFinder();
    initField();
  }
  
  public void periodic() {
    // updateAtflOrigin();
    updateEstimates();
    closestEstimateFinder.update();
    logTagPoses();
  }

  /* I hate null pointer exceptions */
  public void initEstimates(){
    this.targetsPerCam = new ArrayList<>(Collections.nCopies(estimators.size(), new ArrayList<PhotonTrackedTarget>()));
    this.estimates =  new ArrayList<>(Collections.nCopies(estimators.size(), Optional.empty()));
  }

  private void initField(){
    SmartDashboard.putData("vision field", field);
    field.setRobotPose(new Pose2d(-100,-100,new Rotation2d()));
  }

  /**
   * 
   */
  public void updateAtflOrigin(){
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
    for(var i = 0; i < estimators.size(); i++){
      var poseEstimator = estimators.get(i);
      var cam = (PhotonCamera) Constants.VISION.CAMS_AND_TRANS.keySet().toArray()[i];

      var targets = cam.getLatestResult().getTargets();
      targetsPerCam.add(targets);

      var lastPose = swerve.getPose();
      poseEstimator.setReferencePose(lastPose);
      
      var estimate = poseEstimator.update();
      estimates.add(estimate);
    }
  }

  public static boolean isEstimatePresent(Optional<EstimatedRobotPose> estimatedRobotPos) {
    return estimatedRobotPos.isPresent() && estimatedRobotPos.get().estimatedPose != null
        && estimatedRobotPos.get().estimatedPose.getRotation() != null;
  }

  private void initAtfl(){
    try {
      this.atfl = Constants.VISION.TEST_ATFL;
      // this.atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e) {//I really wish we could be doing this in constants.java, not here lol (if you can fix this, plz do)
      DriverStation.reportError("Welp, if ur reading this, imma guess ur getting a nullpointer exception.", true);
    }
  }

  private void initPoseEstimators() {
    estimators = new ArrayList<>();
    for(var camAndTransform : Constants.VISION.CAMS_AND_TRANS.entrySet()){
      estimators.add(new PhotonPoseEstimator(atfl, PoseStrategy.MULTI_TAG_PNP, camAndTransform.getKey(), camAndTransform.getValue()));
    }
  }

  private void initClosestEstimateFinder() {
    this.closestEstimateFinder = new ClosestEstimateFinder(
      Constants.VISION.CAMS_AND_TRANS, 
      getAtfl(),
      swerve.kinematics);
    // this.multiCamPoseEstimator = new MultiCamPoseEstimator(
    //   Constants.VISION.CAMS_AND_TRANS, 
    //   getAtfl(), 
    //   swerve::getWheelPose, 
    //   swerve::getModulePositions, 
    //   swerve::getRotation,
    //   swerve.kinematics, 
    //   swerve::setResetBuffer);
  }

  public void logTagPoses() {
    /* Loop through estimators */
    for(var i = 0; i < estimators.size(); i++){
      var camAndTrans = (Entry<PhotonCamera, Transform3d>) Constants.VISION.CAMS_AND_TRANS.entrySet().toArray()[i];
      var cam = camAndTrans.getKey();
      var robot2Cam = camAndTrans.getValue();
      var optEstimate = estimates.get(i);

      var tagIds = new ArrayList<Integer>();
      var tagPoses = new ArrayList<Pose3d>();
      var robotEstPerTag = new ArrayList<Pose2d>();
      var tagCorners = new ArrayList<TargetCorner>();

      /* Loop through targets */
      for (var target : cam.getLatestResult().getTargets()) {
        var id = target.getFiducialId();
        tagIds.add(id);

        var tagPoseOpt = atfl.getTagPose(id);
        if(tagPoseOpt.isEmpty()){
          tagPoses.add(new Pose3d());
          robotEstPerTag.add(new Pose2d());
          continue;
        }

        var tagPose = tagPoseOpt.get();
        var estRobotPose = tagPose
        .transformBy(target.getBestCameraToTarget().inverse())
        .transformBy(robot2Cam.inverse())
        .toPose2d();

        var corner = target.getDetectedCorners(); // actually 4, but who cares about understandable variable naming lol

        tagPoses.add(tagPose);
        robotEstPerTag.add(estRobotPose);
        tagCorners.addAll(corner);
      }

      /* Prepare values for logging */
      var tagIdsArray = tagIds.stream().mapToLong(Long::valueOf).toArray();
      
      var robotEstPerTagStrs = robotEstPerTag.stream()
      .map((estRobotPose)->AdditionalMathUtils.pos2dToString(estRobotPose, 1))
      .toList().toArray(new String[0]);

      var tagPoseStrs = tagPoses.stream()
      .map((tagPose)->(AdditionalMathUtils.pos2dToString(tagPose.toPose2d(), 1)))
      .toList().toArray(new String[0]);

      var tagCornersStr = tagCorners.stream()
      .map((corner)->AdditionalMathUtils.cornerToString(corner, 1))
      .toList().toArray(new String[0]);

      var estimateStr="invalid";
      if(isEstimatePresent(optEstimate)){
        var estimate = optEstimate.get();
        var pose = estimate.estimatedPose.toPose2d();
        this.field.getObject(cam.getName()).setPose(pose);
        estimateStr=AdditionalMathUtils.pos2dToString(pose, 1);
      }

      var timestamp = cam.getLatestResult().getTimestampSeconds();

      var camMatrixPresent = "invalid";
      if(cam.getCameraMatrix().isPresent()){
        camMatrixPresent="valid";
      }

      var distCoeffPresent = "invalid";
      if(cam.getDistCoeffs().isPresent()){
        distCoeffPresent="valid";
      }

      var statusStr = cam.isConnected() ? "connected" : "disconnected";

      List<Pose3d> akTagPoses = new ArrayList<>();
      List<Integer> akTagIds = new ArrayList<>();
      for (var target : cam.getLatestResult().getTargets()) {
        var swervePose = new Pose3d(swerve.getPose());
        akTagPoses.add(swervePose.transformBy(target.getBestCameraToTarget()));
        akTagIds.add(target.getFiducialId());
      }
      var akTagPosesArray = tagPoses.toArray(new Pose3d[tagPoses.size()]);
      /* May be slightly cursed */
      var akTagIdsArray = tagIds.stream().mapToLong(Long::valueOf).toArray();

      var path = "vision/"+cam.getName()+"/";
      Logger.getInstance().recordOutput(path+"status", statusStr);
      Logger.getInstance().recordOutput(path+"estimate", estimateStr);
      Logger.getInstance().recordOutput(path+"timestamp", timestamp);
      Logger.getInstance().recordOutput(path+"cam matrix", camMatrixPresent);
      Logger.getInstance().recordOutput(path+"dist coeff", distCoeffPresent);

      var targetPath = path + "targets/";
      Logger.getInstance().recordOutput(targetPath+"tag ids", tagIdsArray);
      Logger.getInstance().recordOutput(targetPath+"tag corners", tagCornersStr);
      Logger.getInstance().recordOutput(targetPath+"tag poses", tagPoseStrs);
      Logger.getInstance().recordOutput(targetPath+"robot est per tag", robotEstPerTagStrs);

      var akPath = path + "ak/";
      Logger.getInstance().recordOutput(akPath+"tag poses", akTagPosesArray);
      Logger.getInstance().recordOutput(akPath+"tag ids", akTagIdsArray);
    }
  }
}
