// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.CustomEstimate;
import frc.robot.subsystems.vision.Vision;

public class UpdateSwervePoseEstimator extends CommandBase {
  private Vision vision;

  public enum VisionTrustType {
    MECH_ADV, IRON_PANTHERS, CASSEROLE, TEST, NONE;
  }

  public VisionTrustType visionTrust;
  public SendableChooser<VisionTrustType> trustChooser;

  /** Creates a new UpdateSwervePoseEstimator. */
  public UpdateSwervePoseEstimator(Vision vision) {
    this(vision, VisionTrustType.NONE);
  }

  public UpdateSwervePoseEstimator(Vision vision, VisionTrustType shouldFuseEstimate) {
    this.vision = vision;
    this.visionTrust = shouldFuseEstimate;
    initChooser();
    addRequirements(vision);
  }

  private void initChooser() {
    trustChooser = new SendableChooser<VisionTrustType>();
    for(var visionTrustType : VisionTrustType.values()){
      var key = visionTrustType.name();
      var val = visionTrustType;
      trustChooser.addOption(key, val);
    }

    trustChooser.setDefaultOption(visionTrust.name(), visionTrust);

    SmartDashboard.putData(trustChooser);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionTrust = trustChooser.getSelected();
    for(var optEstimate : vision.estimates){
      if(optEstimate.isEmpty()) continue;
      var estimate = optEstimate.get();
      Matrix<N3, N1> confidence;
      switch(visionTrust){
        case IRON_PANTHERS:
          confidence = Constants.VISION.IRON_PANTHERS_FUNC.apply(estimate);
          estimate.setConfidence(confidence);
          fuseEstimate(estimate);
          break;
         case CASSEROLE:
          confidence = Constants.VISION.ROBOT_CASSEROLE_STDEV;
          estimate.setConfidence(confidence);
          var closeTargets = getAverageTargetToCamDist(estimate) < Constants.FIELD_CONSTANTS.LENGTH * 0.25;
          var isPoseInField = isPoseInField(estimate);
          var isLevel = isLevel();
          var acceptableAmbiguity = estimate.ambiguity < 0.15;
          if(closeTargets && isPoseInField && isLevel && acceptableAmbiguity){
            fuseEstimate(estimate);
          }
          break;
         case MECH_ADV:
          estimate.setConfidence(mechAdvStdDevs(estimate));
          fuseEstimate(estimate);
         break;
         case TEST:
          fuseEstimate(estimate);
          break;
        default:
          break;
      }
    }
  }

  public Matrix<N3, N1> mechAdvStdDevs(CustomEstimate estimate){
          // Calculate average distance to tag
          var avgDist = getAverageTargetToCamDist(estimate);
  
          // Add to vision updates
          var xyStdDevCoefficient = 0.01;
          var thetaStdDevCoefficient = 0.01;

          var numTargets = estimate.targetsUsed.size();
          if(numTargets == 0){
            return VecBuilder.fill(10,10,10);
          }
          double xyStdDev = xyStdDevCoefficient * Math.pow(avgDist, 2.0) / numTargets;
          double thetaStdDev = thetaStdDevCoefficient * Math.pow(avgDist, 2.0) / numTargets;

          return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  public double getAverageTargetToCamDist(CustomEstimate estimate){
    var distSum = 0;
    var targetCount = estimate.targetsUsed.size();
    if(targetCount == 0) return 1000;

    for(var target:estimate.targetsUsed){
      var dist = target.getBestCameraToTarget().getTranslation().getNorm();
      distSum+=dist;
    }
    return distSum / targetCount;
  }

  public boolean isPoseInField(CustomEstimate estimate){
    var xCheck = estimate.best.getX() >= 0 && estimate.best.getX() <=Constants.FIELD_CONSTANTS.LENGTH; 
    var yCheck = estimate.best.getY() >= 0 && estimate.best.getY() <=Constants.FIELD_CONSTANTS.WIDTH;
    return xCheck && yCheck;
  }

  public boolean isLevel(){
    return Math.abs(vision.swerve.getPitch()) < Units.degreesToRadians(10); 
  }

  public void fuseEstimate(CustomEstimate estimate) {
    var poseEstimate = estimate.best.toPose2d();
    var timestamp = estimate.timestampSeconds;
    var confidence = estimate.confidence;
    this.vision.swerve.fuseVisionEstimate(poseEstimate, timestamp, confidence);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
