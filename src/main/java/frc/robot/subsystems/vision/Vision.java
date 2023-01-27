// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private PhotonCamera targetCam;
  private PhotonPoseEstimator PhotonPoseEstimator;

  public Vision() {

    this.targetCam = new PhotonCamera("target");
    updateCamAngle(0);
  }

  public boolean isConnected(){
    return targetCam.isConnected();
  }
  public boolean hasTargets(){
    return targetCam.getLatestResult().hasTargets();
  }

  //Currently getEstimatedGlobalPosFromRotating CAM is not used; instead, getEstimatedGlobalPos is used. 
  public Optional<EstimatedRobotPose> getEstimatedGlobalPosFromRotatingCam(Pose2d prevEstimatedRobotPose, double servoAngle) {
    updateCamAngle(servoAngle);
    return getEstimatedGlobalPos(prevEstimatedRobotPose);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPos(Pose2d prevEstimatedRobotPose) {
    PhotonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    Optional<EstimatedRobotPose> result = PhotonPoseEstimator.update();
    return result;
  }

  private void updateCamAngle(double servoAngle){
    Translation3d camTrans = Constants.Vision.robotToCam.getTranslation();
    Rotation3d servoAngleRot3d = new Rotation3d(0,0,servoAngle);
    Rotation3d camRot = Constants.Vision.robotToCam.getRotation().rotateBy(servoAngleRot3d);
    Transform3d robotToCam = new Transform3d(camTrans, camRot);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(targetCam, robotToCam));
    this.PhotonPoseEstimator = new PhotonPoseEstimator(Constants.Vision.atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, targetCam, Constants.Vision.robotToCam);
  }

  public static boolean isEstimatedRobotPosPresent(Optional<EstimatedRobotPose> estimatedRobotPos){
    return estimatedRobotPos.isPresent() && estimatedRobotPos.get().estimatedPose!=null && estimatedRobotPos.get().estimatedPose.getRotation() != null;
  }

  @Override
  public void periodic() {
  }
}
