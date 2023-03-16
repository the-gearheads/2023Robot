/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.subsystems.vision;

 import edu.wpi.first.apriltag.AprilTag;
 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.math.Pair;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Transform3d;
 import edu.wpi.first.math.geometry.Translation3d;
 import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.MoreMath;
import frc.robot.util.multicamvision.PoseBufferWrapper;
import java.security.cert.CertStore;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
 import java.util.List;
 import java.util.Optional;
 import java.util.Set;
import java.util.stream.Collectors;
import org.apache.commons.io.filefilter.CanExecuteFileFilter;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.VisionEstimation;
 import org.photonvision.targeting.PhotonPipelineResult;
 import org.photonvision.targeting.PhotonTrackedTarget;
 import org.photonvision.targeting.TargetCorner;
import com.fasterxml.jackson.databind.cfg.BaseSettings;
 
 /**
  * The PhotonPoseEstimator class filters or combines readings from all the AprilTags visible at a
  * given timestamp on the field to produce a single robot in field pose, using the strategy set
  * below. Example usage can be found in our apriltagExample example project.
  */
 public class CustomEstimator {
     /** Position estimation strategies that can be used by the {@link PhotonPoseEstimator} class. */
     public enum PoseStrategy {
         /* Multi_TAG_PNP and then disambiguiate based on gyro */
         CLOSEST_TO_GYRO,
         CLOSEST_TO_BUFFER_POSE,
         CLOSEST_TO_GYRO_PRIME,
         CLOSEST_TO_BUFFER_POSE_PRIME,
         MIN_AMBIGUITY
     }
 
     private AprilTagFieldLayout fieldTags;
     private PoseStrategy primaryStrategy;
     private final PhotonCamera camera;
     private Transform3d robotToCamera;
 
     protected double poseCacheTimestampSeconds = -1;
     private final Set<Integer> reportedErrors = new HashSet<>();
 
     /**
      * Create a new PhotonPoseEstimator.
      *
      * @param fieldTags A WPILib {@link AprilTagFieldLayout} linking AprilTag IDs to Pose3d objects
      *     with respect to the FIRST field using the <a
      *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#field-coordinate-system">Field
      *     Coordinate System</a>.
      * @param strategy The strategy it should use to determine the best pose.
      * @param camera PhotonCamera
      * @param robotToCamera Transform3d from the center of the robot to the camera mount position (ie,
      *     robot ➔ camera) in the <a
      *     href="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system">Robot
      *     Coordinate System</a>.
      */
     public CustomEstimator(
             AprilTagFieldLayout fieldTags,
             PoseStrategy strategy,
             PhotonCamera camera,
             Transform3d robotToCamera) {
         this.fieldTags = fieldTags;
         this.primaryStrategy = strategy;
         this.camera = camera;
         this.robotToCamera = robotToCamera;
     }
 
     /** Invalidates the pose cache. */
     private void invalidatePoseCache() {
         poseCacheTimestampSeconds = -1;
     }
 
     private void checkUpdate(Object oldObj, Object newObj) {
         if (oldObj != newObj && oldObj != null && !oldObj.equals(newObj)) {
             invalidatePoseCache();
         }
     }
 
     /**
      * Get the AprilTagFieldLayout being used by the PositionEstimator.
      *
      * @return the AprilTagFieldLayout
      */
     public AprilTagFieldLayout getFieldTags() {
         return fieldTags;
     }
 
     /**
      * Set the AprilTagFieldLayout being used by the PositionEstimator.
      *
      * @param fieldTags the AprilTagFieldLayout
      */
     public void setFieldTags(AprilTagFieldLayout fieldTags) {
         checkUpdate(this.fieldTags, fieldTags);
         this.fieldTags = fieldTags;
     }
 
     /**
      * Get the Position Estimation Strategy being used by the Position Estimator.
      *
      * @return the strategy
      */
     public PoseStrategy getPrimaryStrategy() {
         return primaryStrategy;
     }
 
     /**
      * Set the Position Estimation Strategy used by the Position Estimator.
      *
      * @param strategy the strategy to set
      */
     public void setPrimaryStrategy(PoseStrategy strategy) {
         checkUpdate(this.primaryStrategy, strategy);
         this.primaryStrategy = strategy;
     }

     /** @return The current transform from the center of the robot to the camera mount position */
     public Transform3d getRobotToCameraTransform() {
         return robotToCamera;
     }
 
     /**
      * Useful for pan and tilt mechanisms and such.
      *
      * @param robotToCamera The current transform from the center of the robot to the camera mount
      *     position
      */
     public void setRobotToCameraTransform(Transform3d robotToCamera) {
         this.robotToCamera = robotToCamera;
     }
 
     /**
      * Poll data from the configured cameras and update the estimated position of the robot. Returns
      * empty if there are no cameras set or no targets were found from the cameras.
      *
      * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
      *     the estimate
      */
     public Optional<CustomEstimate> update() {
         if (camera == null) {
             DriverStation.reportError("[PhotonPoseEstimator] Missing camera!", false);
             return Optional.empty();
         }
 
         PhotonPipelineResult cameraResult = camera.getLatestResult();
 
         return update(cameraResult);
     }
 
     /**
      * Updates the estimated position of the robot. Returns empty if there are no cameras set or no
      * targets were found from the cameras.
      *
      * @param cameraResult The latest pipeline result from the camera
      * @return an EstimatedRobotPose with an estimated pose, and information about the camera(s) and
      *     pipeline results used to create the estimate
      */
     public Optional<CustomEstimate> update(PhotonPipelineResult cameraResult) {
         // Time in the past -- give up, since the following if expects times > 0
         if (cameraResult.getTimestampSeconds() < 0) {
             return Optional.empty();
         }
 
         // If the pose cache timestamp was set, and the result is from the same timestamp, return an
         // empty result
         if (poseCacheTimestampSeconds > 0
                 && Math.abs(poseCacheTimestampSeconds - cameraResult.getTimestampSeconds()) < 1e-6) {
             return Optional.empty();
         }
 
         // Remember the timestamp of the current result used
         poseCacheTimestampSeconds = cameraResult.getTimestampSeconds();
 
         // If no targets seen, trivial case -- return empty result
         if (!cameraResult.hasTargets()) {
             return Optional.empty();
         }
 
         return update(cameraResult, this.primaryStrategy);
     }
 
     private Optional<CustomEstimate> update(
             PhotonPipelineResult cameraResult, PoseStrategy strat) {
        
         cameraResult = filterTargets(cameraResult);
         Optional<CustomEstimate> estimatedPose;
         switch (strat) {
             case CLOSEST_TO_GYRO:
                 estimatedPose = closestToGyroStrategy(cameraResult);
                 break;
             case CLOSEST_TO_BUFFER_POSE:
                 estimatedPose = closestToBufferPoseStrategy(cameraResult);
                 break;
             case CLOSEST_TO_GYRO_PRIME:
                 estimatedPose = closestToGyroPrimeStrategy(cameraResult);
                 break;
             case CLOSEST_TO_BUFFER_POSE_PRIME:
                 estimatedPose = closestToBufferPosePrimeStrategy(cameraResult);
                 break;
             case MIN_AMBIGUITY:
                 estimatedPose = minAmbiguityStrategy(cameraResult);
             default:
                 DriverStation.reportError(
                         "[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false);
                 return Optional.empty();
         }

         return estimatedPose;
     }
 
     private PhotonPipelineResult filterTargets(PhotonPipelineResult cameraResult) {
        var filteredTargets = cameraResult.targets.stream().filter((target)->{
            Optional<Pose3d> targetPose = fieldTags.getTagPose(target.getFiducialId());
            return targetPose.isPresent();
        }).collect(Collectors.toList());
        
        var filteredRes = new PhotonPipelineResult(cameraResult.getLatencyMillis(), filteredTargets);
        filteredRes.setTimestampSeconds(cameraResult.getTimestampSeconds());
        return filteredRes; 
    }

    private Optional<CustomEstimate> minAmbiguityStrategy(PhotonPipelineResult cameraResult){
        Optional<ExtendedCustomEstimate> optEstimate = getMultiTagEstimate(cameraResult);
        if(optEstimate.isEmpty()){
            return Optional.empty();
        }
        var estimate = optEstimate.get();

        if(estimate.targetsUsed.size() > 1){
            return Optional.of(estimate.getBestEstimate());
        }else{
            if(estimate.ambiguity < 0.15){
                return Optional.of(estimate.getBestEstimate());
            }else{
                return Optional.empty();
            }
        }
    }

    private Optional<CustomEstimate> closestToGyroPrimeStrategy(PhotonPipelineResult cameraResult){
        Optional<ExtendedCustomEstimate> optEstimate = getMultiTagEstimate(cameraResult);
        if(optEstimate.isEmpty()){
            return Optional.empty();
        }
        var estimate = optEstimate.get();

        if(estimate.targetsUsed.size() > 1){
            return Optional.of(estimate.getBestEstimate());
        }else{
            var contemporaryGyroRad = PoseBufferWrapper.getPoseInstance().getSample(estimate.timestampSeconds).get().gyroAngle.getRadians();
        
            var bestRad = estimate.best.getRotation().getZ();
            var altRad = estimate.alt.getRotation().getZ();

            var closestToBestRad = MoreMath.getClosestRad(bestRad, contemporaryGyroRad);
            var closestToAltRad = MoreMath.getClosestRad(altRad, contemporaryGyroRad);

            var distFromBest = Math.abs(bestRad - closestToBestRad);
            var distFromAlt = Math.abs(altRad - closestToAltRad);

            if(distFromBest <= distFromAlt){
                return Optional.of(estimate.getBestEstimate());
            }else{
                return Optional.of(estimate.getAltEstimate());
            }
        }
    }

    private Optional<CustomEstimate> closestToBufferPosePrimeStrategy(PhotonPipelineResult cameraResult){
        Optional<ExtendedCustomEstimate> optEstimate = getMultiTagEstimate(cameraResult);
        if(optEstimate.isEmpty()){
            return Optional.empty();
        }
        var estimate = optEstimate.get();

        if(estimate.targetsUsed.size() > 1){
            return Optional.of(estimate.getBestEstimate());
        }else{
            var contemporaryPose = new Pose3d(PoseBufferWrapper.getPoseInstance().getSample(estimate.timestampSeconds).get().poseMeters);
            var bestDistance = calculateDifference(contemporaryPose, estimate.best);
            var altDistance = calculateDifference(contemporaryPose, estimate.alt);

            if(bestDistance <= altDistance){
                return Optional.of(estimate.getBestEstimate());
            }else{
                return Optional.of(estimate.getAltEstimate());
            }
        }
    }

    private Optional<CustomEstimate> closestToGyroStrategy(PhotonPipelineResult cameraResult) {
        Optional<ExtendedCustomEstimate> optEstimate = getMultiTagEstimate(cameraResult);
        if(optEstimate.isEmpty()){
            return Optional.empty();
        }
        var estimate = optEstimate.get();

        var contemporaryGyroRad = PoseBufferWrapper.getPoseInstance().getSample(estimate.timestampSeconds).get().gyroAngle.getRadians();
        
        var bestRad = estimate.best.getRotation().getZ();
        var altRad = estimate.alt.getRotation().getZ();

        var closestToBestRad = MoreMath.getClosestRad(bestRad, contemporaryGyroRad);
        var closestToAltRad = MoreMath.getClosestRad(altRad, contemporaryGyroRad);

        var distFromBest = Math.abs(bestRad - closestToBestRad);
        var distFromAlt = Math.abs(altRad - closestToAltRad);

        if(distFromBest <= distFromAlt){
            return Optional.of(estimate.getBestEstimate());
        }else{
            return Optional.of(estimate.getAltEstimate());
        }
    }

    private Optional<CustomEstimate> closestToBufferPoseStrategy(PhotonPipelineResult cameraResult) {
        Optional<ExtendedCustomEstimate> optEstimate = getMultiTagEstimate(cameraResult);
        if(optEstimate.isEmpty()){
            return Optional.empty();
        }
        var estimate = optEstimate.get();

        var contemporaryPose = new Pose3d(PoseBufferWrapper.getPoseInstance().getSample(estimate.timestampSeconds).get().poseMeters);
        var bestDistance = calculateDifference(contemporaryPose, estimate.best);
        var altDistance = calculateDifference(contemporaryPose, estimate.alt);

        if(bestDistance <= altDistance){
            return Optional.of(estimate.getBestEstimate());
        }else{
            return Optional.of(estimate.getAltEstimate());
        }
    }

private Optional<ExtendedCustomEstimate> getMultiTagEstimate(PhotonPipelineResult result){
    ExtendedCustomEstimate estimate;
    var numTargets = result.getTargets().size();
    if(numTargets == 0){
        return Optional.empty();
    }else if(numTargets == 1){
        var target = result.targets.get(0);

        /* make sure target exists and is valid */
        var targetPosition = fieldTags.getTagPose(target.getFiducialId()).get();

        var best = targetPosition
        .transformBy(target.getBestCameraToTarget().inverse())
        .transformBy(robotToCamera.inverse());

        var alt = targetPosition
        .transformBy(target.getAlternateCameraToTarget().inverse())
        .transformBy(robotToCamera.inverse());

        estimate = new ExtendedCustomEstimate(best, alt, target.getPoseAmbiguity(),
         result.getTimestampSeconds(), result.getTargets());
        
         return Optional.of(estimate);
    }else{
        var customEstimateOpt = innerMultiTag(result);
        return customEstimateOpt;
    }
}
private Optional<ExtendedCustomEstimate> innerMultiTag(PhotonPipelineResult result) {
         // Arrays we need declared up front
         var visCorners = new ArrayList<TargetCorner>();
         var knownVisTags = new ArrayList<AprilTag>();
         var fieldToCams = new ArrayList<Pose3d>();
         var fieldToCamsAlt = new ArrayList<Pose3d>();
 
         for (var target : result.getTargets()) {
             visCorners.addAll(target.getDetectedCorners());
 
             var tagPoseOpt = fieldTags.getTagPose(target.getFiducialId());
             if (tagPoseOpt.isEmpty()) {
                 reportFiducialPoseError(target.getFiducialId());
                 continue;
             }
 
             var tagPose = tagPoseOpt.get();
 
             // actual layout poses of visible tags -- not exposed, so have to recreate
             knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));
 
             fieldToCams.add(tagPose.transformBy(target.getBestCameraToTarget().inverse()));
             fieldToCamsAlt.add(tagPose.transformBy(target.getAlternateCameraToTarget().inverse()));
         }
 
         var cameraMatrixOpt = camera.getCameraMatrix();
         var distCoeffsOpt = camera.getDistCoeffs();
         boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();
 
         // multi-target solvePNP
         if (hasCalibData) {
             var cameraMatrix = cameraMatrixOpt.get();
             var distCoeffs = distCoeffsOpt.get();
             var pnpResults =
                     VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
             var best =
                     new Pose3d()
                             .plus(pnpResults.best) // field-to-camera
                             .plus(robotToCamera.inverse()); // field-to-robot
             var alt = new Pose3d()
             .plus(pnpResults.alt) // field-to-camera
             .plus(robotToCamera.inverse()); // field-to-robot
 
             return Optional.of(
                     new ExtendedCustomEstimate(best, alt, pnpResults.bestReprojErr / pnpResults.altReprojErr, result.getTimestampSeconds(), result.getTargets()));
         } else {
             // TODO fallback strategy? Should we just always do solvePNP?
             return Optional.empty();
         }
        }
 
     /**
      * Difference is defined as the vector magnitude between the two poses
      *
      * @return The absolute "difference" (>=0) between two Pose3ds.
      */
     private double calculateDifference(Pose3d x, Pose3d y) {
         return x.getTranslation().getDistance(y.getTranslation());
     }
 
     private void reportFiducialPoseError(int fiducialId) {
         if (!reportedErrors.contains(fiducialId)) {
             DriverStation.reportError(
                     "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + fiducialId, false);
             reportedErrors.add(fiducialId);
         }
     }
 }
 