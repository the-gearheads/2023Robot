// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.multicamvision.PoseBufferWrapper;

/** Add your docs here. */
public class EstimatorWrapper {
    public enum WrapperStrategy{
        LOWEST_AMBIGUITY(PoseStrategy.MULTI_TAG_PNP, PoseStrategy.LOWEST_AMBIGUITY),
        CLOSEST_TO_BUFFER(PoseStrategy.MULTI_TAG_PNP, PoseStrategy.LOWEST_AMBIGUITY), 
        CLOSEST_TO_GYRO(PoseStrategy.MULTI_TAG_PNP, PoseStrategy.LOWEST_AMBIGUITY);

        
        private PoseStrategy secondaryStrat;
        private PoseStrategy primaryStrat;

        WrapperStrategy(PoseStrategy primaryStrat, PoseStrategy secondaryStrat){
            this.primaryStrat=primaryStrat;
            this.secondaryStrat = secondaryStrat;    
        }
    }

    private Transform3d robot2Cam;
    private PhotonCamera cam;
    private PhotonPoseEstimator estimator;
    private WrapperStrategy strat;

    public EstimatorWrapper(PhotonCamera cam, Transform3d robot2Cam, AprilTagFieldLayout atfl, WrapperStrategy strat){
        this.cam = cam;
        this.robot2Cam = robot2Cam;
        this.estimator = new PhotonPoseEstimator(atfl, strat.primaryStrat, cam, robot2Cam);
        this.estimator.setMultiTagFallbackStrategy(strat.secondaryStrat);
        this.strat = strat;
    }

    public Optional<EstimatedRobotPose> getEstimate(){

        Optional<EstimatedRobotPose> optEstimate = Optional.empty();

        switch(strat){
            case LOWEST_AMBIGUITY:
                optEstimate = lowestAmbiguityStrategy();
                break;
            case CLOSEST_TO_GYRO:
                optEstimate = closestToGyroStrategy();
                break;
        }
    }

    public Optional<EstimatedRobotPose> lowestAmbiguityStrategy(){
        var optEstimate = this.estimator.update();

        return optEstimate;
    }
    
    public Optional<EstimatedRobotPose> closestToGyroStrategy(){
        var optEstimate = this.estimator.update();

        if(!Vision.isEstimatePresent(optEstimate)){
            return optEstimate;
        }

        if(cam.getLatestResult().getTargets().size() >= 2){
            
        }
        return optEstimate;
    }

    public Pose2d getContemporaryPose(EstimatedRobotPose estimate) {
        return PoseBufferWrapper.getInstance().getSample(estimate.timestampSeconds).get().poseMeters;    
    }

    public Rotation2d getContemporaryGyroAngle(EstimatedRobotPose estimate){
        return PoseBufferWrapper.getInstance().getSample(estimate.timestampSeconds).get().gyroAngle;
    }

    private EstimatedRobotPose getAltEstimate(EstimatedRobotPose bestEstimate) {
        // Arrays we need declared up front
        var visCorners = new ArrayList<TargetCorner>();
        var knownVisTags = new ArrayList<AprilTag>();
        var fieldToCams = new ArrayList<Pose3d>();
        var fieldToCamsAlt = new ArrayList<Pose3d>();
  
        var result = this.cam.getLatestResult();
        var fieldTags = estimator.getFieldTags();
  
        if (result.getTargets().size() < 2) {
            // Run fallback strategy instead
            var tag = result.getTargets().get(0);
            Pose3d fieldToTag = fieldTags.getTagPose(tag.getFiducialId()).get();
            var altCamToTag = result.targets.get(0).getAlternateCameraToTarget();
            var fieldToRobot = fieldToTag
            .plus(altCamToTag.inverse())
            .plus(robot2Cam.inverse());
            return new EstimatedRobotPose(fieldToRobot, result.getTimestampSeconds(), List.of(tag));
        }
  
        for (var target : bestEstimate.targetsUsed) {
            visCorners.addAll(target.getDetectedCorners());
  
            var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
            var tagPose = tagPoseOpt.get();
  
            // actual layout poses of visible tags -- not exposed, so have to recreate
            knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));
  
            fieldToCams.add(tagPose.transformBy(target.getBestCameraToTarget().inverse()));
            fieldToCamsAlt.add(tagPose.transformBy(target.getAlternateCameraToTarget().inverse()));
        }
  
        var cameraMatrixOpt = cam.getCameraMatrix();
        var distCoeffsOpt = cam.getDistCoeffs();
        boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();
  
        // multi-target solvePNP
        if (hasCalibData) {
            var cameraMatrix = cameraMatrixOpt.get();
            var distCoeffs = distCoeffsOpt.get();
            var pnpResults =
                    VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
            var alt = new Pose3d()
            .plus(pnpResults.alt) // field-to-camera
            .plus(robot2Cam.inverse()); // field-to-robot
  
            return new EstimatedRobotPose(alt, result.getTimestampSeconds(), result.getTargets());
        } else {
            // TODO fallback strategy? Should we just always do solvePNP?
            return null;
        }
    }
}
