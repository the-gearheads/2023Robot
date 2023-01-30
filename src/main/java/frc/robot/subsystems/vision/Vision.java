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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private PhotonCamera targetCam;
  private PhotonPoseEstimator photonPoseEstimator;

  public Vision() { 
    this.targetCam = new PhotonCamera("target");
    initializePoseEstimator();
  }

  public boolean isConnected(){
    return targetCam.isConnected();
  }
  public boolean hasTargets(){
    return targetCam.getLatestResult().hasTargets();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPos(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  private void initializePoseEstimator(){
    //Create camList
    Transform3d robotToCam = Constants.Vision.robotToCam;
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<>(targetCam, robotToCam));

    AprilTagFieldLayout atfl=null;
    try{
      atfl = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    }catch(Exception e){//I really wish we could be doing this in constants.java, not here lol (if you can fix this, plz do)
      DriverStation.reportError("Welp, if ur reading this, imma guess ur getting a nullpointer exception.", true);
    }
    this.photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, targetCam, Constants.Vision.robotToCam);
  }

  public static boolean isEstimatedRobotPosPresent(Optional<EstimatedRobotPose> estimatedRobotPos){
    return estimatedRobotPos.isPresent() && estimatedRobotPos.get().estimatedPose!=null && estimatedRobotPos.get().estimatedPose.getRotation() != null;
  }
}
