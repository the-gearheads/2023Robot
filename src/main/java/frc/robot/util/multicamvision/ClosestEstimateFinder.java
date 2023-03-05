// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.multicamvision;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;

/** Add your docs here. */
public class ClosestEstimateFinder {
  public Matrix<N3, N3> cameraMatrix;
  public Matrix<N5, N1> distCoeffs;
  private PoseBufferWrapper poseBuffer;

  public ClosestEstimateFinder(HashMap<PhotonCamera, Transform3d> camsAndtransforms, AprilTagFieldLayout atfl,
      SwerveDriveKinematics kinematics) {
    this(() -> new Pose2d(), () -> {
      SwerveModulePosition[] modPoses = new SwerveModulePosition[4];
      for (var i = 0; i < 4; i++) {
        modPoses[i] = new SwerveModulePosition();
      }
      return modPoses;
    }, () -> new Rotation2d(), kinematics, (temp) -> {
    });
  }

  public ClosestEstimateFinder(HashMap<PhotonCamera, Transform3d> camsAndtransforms, AprilTagFieldLayout atfl,
      Supplier<Pose2d> getWheelPose, Supplier<SwerveModulePosition[]> getModulePositions,
      Supplier<Rotation2d> getRotation, SwerveDriveKinematics kinematics, Consumer<Runnable> setResetBuffer) {
    this(getWheelPose, getModulePositions, getRotation, kinematics, setResetBuffer);
  }

  public ClosestEstimateFinder(Supplier<Pose2d> getWheelPose, Supplier<SwerveModulePosition[]> getModulePositions,
      Supplier<Rotation2d> getRotation, SwerveDriveKinematics kinematics, Consumer<Runnable> setResetBuffer) {
    this.poseBuffer = new PoseBufferWrapper(getWheelPose, getModulePositions, getRotation, kinematics, setResetBuffer);
  }

  public void update() {
    this.poseBuffer.update();
  }

  // public Optional<EstimatedRobotPose> getClosestEstimate(PhotonPoseEstimator estimator, Optional<EstimatedRobotPose> optEstimate) {
  //   if(!Vision.isEstimatePresent(optEstimate)){
  //       return optEstimate;
  //   }
  //   var estimate = optEstimate.get();
  //   var altEstimate = getAltEstimate(estimator, estimate);

  //   var syncEstimate = syncEstimate(estimate);
  //   var syncAltEstimate = syncEstimate(altEstimate);

  //   var diff = diffFromWheelPose(syncEstimate);
  //   var altDiff = diffFromWheelPose(syncAltEstimate);
  //   if(diff<altDiff){
  //       return Optional.of(estimate);
  //   }else{
  //       return Optional.of(altEstimate);
  //   }
  // }

  // public EstimatedRobotPose syncEstimate(EstimatedRobotPose estimate) {
  //   var contempWheelPose = poseBuffer.getSample(estimate.timestampSeconds).get().poseMeters;
  //   var currentWheelPose = poseBuffer.getSample(Timer.getFPGATimestamp()).get().poseMeters;
  //   var twist2d = currentWheelPose.log(contempWheelPose);
  //   var twist3d = new Twist3d(twist2d.dx, twist2d.dy, 0, 0, 0, twist2d.dtheta);

  //   var syncPose = estimate.estimatedPose.exp(twist3d);
  //   var syncEstimate = new EstimatedRobotPose(syncPose, estimate.timestampSeconds, estimate.targetsUsed);

  //   return syncEstimate;
  // }

  // private EstimatedRobotPose getAltEstimate(PhotonPoseEstimator estimator, EstimatedRobotPose bestEstimate) {
  //     // Arrays we need declared up front
  //     var visCorners = new ArrayList<TargetCorner>();
  //     var knownVisTags = new ArrayList<AprilTag>();
  //     var fieldToCams = new ArrayList<Pose3d>();
  //     var fieldToCamsAlt = new ArrayList<Pose3d>();

  //     var result = estimator.getCam().getLatestResult();
  //     var fieldTags = estimator.getFieldTags();
  //     var camera = estimator.getCam();
  //     var robotToCamera = estimator.getRobot2Cam();

  //     if (result.getTargets().size() < 2) {
  //         // Run fallback strategy instead
  //         var tag = result.getTargets().get(0);
  //         Pose3d fieldToTag = fieldTags.getTagPose(tag.getFiducialId()).get();
  //         var altCamToTag = result.targets.get(0).getAlternateCameraToTarget();
  //         var fieldToRobot = fieldToTag
  //         .plus(altCamToTag.inverse())
  //         .plus(estimator.getRobot2Cam().inverse());
  //         return new EstimatedRobotPose(fieldToRobot, result.getTimestampSeconds(), List.of(tag));
  //     }

  //     for (var target : bestEstimate.targetsUsed) {
  //         visCorners.addAll(target.getDetectedCorners());

  //         var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
  //         var tagPose = tagPoseOpt.get();

  //         // actual layout poses of visible tags -- not exposed, so have to recreate
  //         knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));

  //         fieldToCams.add(tagPose.transformBy(target.getBestCameraToTarget().inverse()));
  //         fieldToCamsAlt.add(tagPose.transformBy(target.getAlternateCameraToTarget().inverse()));
  //     }

  //     var cameraMatrixOpt = camera.getCameraMatrix();
  //     var distCoeffsOpt = camera.getDistCoeffs();
  //     boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();

  //     // multi-target solvePNP
  //     if (hasCalibData) {
  //         var cameraMatrix = cameraMatrixOpt.get();
  //         var distCoeffs = distCoeffsOpt.get();
  //         var pnpResults =
  //                 VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
  //         var alt = new Pose3d()
  //         .plus(pnpResults.alt) // field-to-camera
  //         .plus(robotToCamera.inverse()); // field-to-robot

  //         return new EstimatedRobotPose(alt, result.getTimestampSeconds(), result.getTargets());
  //     } else {
  //         // TODO fallback strategy? Should we just always do solvePNP?
  //         return null;
  //     }
  // }

  // private double calculateDifference(Pose3d x, Pose3d y) {
  //     return x.getTranslation().getDistance(y.getTranslation());
  // }

  // private double diffFromWheelPose(EstimatedRobotPose estimate){
  //     var currentWheelPose = poseBuffer.getSample(estimate.timestampSeconds).get().poseMeters;
  //     return calculateDifference(new Pose3d(currentWheelPose), estimate.estimatedPose);
  // }
}
