// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private PhotonCamera targetCam;
  private RobotPoseEstimator robotPoseEstimator;

  public Vision() {

    this.targetCam = new PhotonCamera("target");
    updateCamAngle(0);
  }

  //Cam Functions
  public boolean isConnected(){
    return targetCam.isConnected();
  }
  public boolean hasTargets(){
    return targetCam.getLatestResult().hasTargets();
  }
  //Currently getEstimatedGlobalPosFromRotating CAM is not used; instead, getEstimatedGlobalPos is used. 
  public Pair<Pose2d, Double> getEstimatedGlobalPosFromRotatingCam(Pose2d prevEstimatedRobotPose, double servoAngle) {
    updateCamAngle(servoAngle);
    return getEstimatedGlobalPos(prevEstimatedRobotPose);
  }
  public Pair<Pose2d, Double> getEstimatedGlobalPos(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent() && result.get().getFirst()!=null&&result.get().getFirst().getRotation()!=null) {//the 2nd and 3rd conditions avoid nullpointer exceptions. 
      SmartDashboard.putBoolean("Vision/ResultPresent", true);
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        SmartDashboard.putBoolean("Vision/ResultPresent", false);
        return new Pair<Pose2d, Double>(prevEstimatedRobotPose, 0.0);
    }
  }
  private void updateCamAngle(double servoAngle){
    Translation3d camTrans=Constants.Vision.robotToCam.getTranslation();
    Rotation3d servoAngleRot3d = new Rotation3d(0,0,servoAngle);
    Rotation3d camRot = Constants.Vision.robotToCam.getRotation().rotateBy(servoAngleRot3d);
    Transform3d robotToCam= new Transform3d(camTrans, camRot);

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(targetCam, robotToCam));
    this.robotPoseEstimator=new RobotPoseEstimator(Constants.Vision.atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  @Override
  public void periodic() {
  }

  private double calculateDifference(Pose3d x, Pose3d y) {
    return x.getTranslation().getDistance(y.getTranslation());
  }
  private Pair<Pose3d, Double> closestToReferencePoseStrategy(Pose2d referencePose, double servoAngle) {
    //Create Cam List
    Translation3d camTrans=Constants.Vision.robotToCam.getTranslation();
    Rotation3d servoAngleRot3d = new Rotation3d(0,0,servoAngle);
    Rotation3d camRot = Constants.Vision.robotToCam.getRotation().rotateBy(servoAngleRot3d);
    Transform3d robotToCam= new Transform3d(camTrans, camRot);

    var cameras = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    cameras.add(new Pair<PhotonCamera, Transform3d>(targetCam, robotToCam));

    if (referencePose == null) {
        DriverStation.reportError(
                "[RobotPoseEstimator] Tried to use reference pose strategy without setting the reference!",
                false);
        return null;
    }
    double smallestDifference = 10e9;
    double latency = 0;
    Pose3d pose = null;
    double angle = 0;
    for (int i = 0; i < cameras.size(); i++) {
        Pair<PhotonCamera, Transform3d> p = cameras.get(i);
        var result = p.getFirst().getLatestResult();
        List<PhotonTrackedTarget> targets = result.targets;
        for (int j = 0; j < targets.size(); j++) {
            PhotonTrackedTarget target = targets.get(j);
            Optional<Pose3d> fiducialPose = Constants.Vision.atfl.getTagPose(target.getFiducialId());
            if (fiducialPose.isEmpty()) {
                continue;
            }
            Pose3d targetPose = fiducialPose.get();
            Pose3d botBestPose =
                    targetPose
                            .transformBy(target.getAlternateCameraToTarget().inverse())
                            .transformBy(p.getSecond().inverse());
            Pose3d botAltPose =
                    targetPose
                            .transformBy(target.getBestCameraToTarget().inverse())
                            .transformBy(p.getSecond().inverse());
            double alternativeDifference = Math.abs(calculateDifference(new Pose3d(referencePose), botAltPose));
            double bestDifference = Math.abs(calculateDifference(new Pose3d(referencePose), botBestPose));
            if (alternativeDifference < smallestDifference) {
                smallestDifference = alternativeDifference;
                pose = botAltPose;
                latency = result.getLatencyMillis();
                angle=target.getBestCameraToTarget().getRotation().getZ();
            }
            if (bestDifference < smallestDifference) {
                smallestDifference = bestDifference;
                pose = botBestPose;
                latency = result.getLatencyMillis();
                angle=target.getBestCameraToTarget().getRotation().getZ();
            }
        }
    }
    return Pair.of(pose, latency);
}
}
