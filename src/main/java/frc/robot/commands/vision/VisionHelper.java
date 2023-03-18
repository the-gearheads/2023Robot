// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.vision.CustomEstimate;
import frc.robot.subsystems.vision.Vision;

/** Add your docs here. */
public class VisionHelper {

  public static Matrix<N3, N1> mechAdvStdDevs(CustomEstimate estimate) {
    // Calculate average distance to tag
    var avgDist = getAverageTargetToCamDist(estimate);

    // Add to vision updates
    var xyStdDevCoefficient = 0.01;
    var thetaStdDevCoefficient = 0.01;

    var numTargets = estimate.targetsUsed.size();
    if (numTargets == 0) {
      return VecBuilder.fill(10, 10, 10);
    }
    double xyStdDev = xyStdDevCoefficient * Math.pow(avgDist, 2.0) / numTargets;
    double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDist, 2.0) / numTargets;

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public static Matrix<N3, N1> ironPanthersStdDevs(CustomEstimate estimate) {
    double POSE_AMBIGUITY_SHIFTER = 0.2;
    double POSE_AMBIGUITY_MULTIPLIER = 4;
    double NOISY_DISTANCE_METERS = 2.5;
    double DISTANCE_WEIGHT = 7;
    int TAG_PRESENCE_WEIGHT = 10;
    Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1()).fill(
        // if these numbers are less than one, multiplying will do bad things
        1, // x
        1, // y
        1 * Math.PI // theta
    );

    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimate.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance)
        smallestDistance = distance;
    }
    double poseAmbiguityFactor = estimate.targetsUsed.size() != 1 ? 1
        : Math.max(1,
            (estimate.targetsUsed.get(0).getPoseAmbiguity() + POSE_AMBIGUITY_SHIFTER) * POSE_AMBIGUITY_MULTIPLIER);
    double confidenceMultiplier = Math.max(1,
        (Math.max(1, Math.max(0, smallestDistance - NOISY_DISTANCE_METERS) * DISTANCE_WEIGHT) * poseAmbiguityFactor)
            / (1 + ((estimate.targetsUsed.size() - 1) * TAG_PRESENCE_WEIGHT)));

    var confidence = VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    return confidence;
  }

  public static double getAverageTargetToCamDist(CustomEstimate estimate) {
    var distSum = 0;
    var targetCount = estimate.targetsUsed.size();
    if (targetCount == 0)
      return 1000;

    for (var target : estimate.targetsUsed) {
      var dist = target.getBestCameraToTarget().getTranslation().getNorm();
      distSum += dist;
    }
    return distSum / targetCount;
  }

  public static boolean isPoseInField(CustomEstimate estimate) {
    var xCheck = estimate.best.getX() >= 0 && estimate.best.getX() <= Constants.FIELD_CONSTANTS.LENGTH;
    var yCheck = estimate.best.getY() >= 0 && estimate.best.getY() <= Constants.FIELD_CONSTANTS.WIDTH;
    return xCheck && yCheck;
  }

  public static boolean isLevel(Vision vision) {
    return Math.abs(vision.swerve.getPitch()) < Units.degreesToRadians(10);
  }

  public static final Matrix<N3, N1> ROBOT_CASSEROLE_STDEV = VecBuilder.fill(0.9, 0.9, 0.009);

  public static boolean casseroleShouldFuse(CustomEstimate estimate, Vision vision){
    var closeTargets = VisionHelper.getAverageTargetToCamDist(estimate) < Constants.FIELD_CONSTANTS.LENGTH * 0.25;
    var isPoseInField = VisionHelper.isPoseInField(estimate);
    var isLevel = VisionHelper.isLevel(vision);
    var acceptableAmbiguity = estimate.ambiguity < 0.15;
    if (closeTargets && isPoseInField && isLevel && acceptableAmbiguity) {
      return true;
    }
    return false;
  }
}
