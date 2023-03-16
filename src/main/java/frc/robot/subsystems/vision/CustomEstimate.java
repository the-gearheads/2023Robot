// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class CustomEstimate {
        /** The estimated time the frame used to derive the robot pose was taken */
        public final double timestampSeconds;
    
        /** A list of the targets used to compute this pose */
        public final List<PhotonTrackedTarget> targetsUsed;

        public final Pose3d best;

        public Matrix<N3, N1> confidence;

        public final double ambiguity;
    
        /**
         * Constructs an EstimatedRobotPose
         *
         * @param estimatedPose estimated pose
         * @param timestampSeconds timestamp of the estimate
         */
        public CustomEstimate(
                Pose3d best, double ambiguity, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed, Matrix<N3, N1> confidence) {
            this.best = best;
            this.ambiguity = ambiguity;
            this.timestampSeconds = timestampSeconds;
            this.targetsUsed = targetsUsed;
            this.confidence=confidence;
        }

        public CustomEstimate(
            Pose3d best, double ambiguity, double timestampSeconds, List<PhotonTrackedTarget> targetsUsed) {
        this(best, ambiguity, timestampSeconds, targetsUsed, VecBuilder.fill(0.9, 0.9, 0.9));
        }


        public void setConfidence(Matrix<N3, N1> confidence){
            this.confidence=confidence;
        }
}
