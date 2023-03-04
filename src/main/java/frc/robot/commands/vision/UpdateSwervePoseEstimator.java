// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.Vision;

public class UpdateSwervePoseEstimator extends CommandBase {
  private Vision vision;
  public enum ShouldFuseEstimate{
    YES, NO;
  }
  public ShouldFuseEstimate shouldFuseEstimate; 

  /** Creates a new UpdateSwervePoseEstimator. */
  public UpdateSwervePoseEstimator(Vision vision) {
    this(vision, ShouldFuseEstimate.YES);
  }
  public UpdateSwervePoseEstimator(Vision vision, ShouldFuseEstimate shouldFuseEstimate) {
    this.vision = vision;
    this.shouldFuseEstimate=shouldFuseEstimate;

    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Vision/Use Closest Estimate", false);
    SmartDashboard.putBoolean("Vision/Vary StDev", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var useClosestEstimate = SmartDashboard.putBoolean("Vision/Use Closest Estimate", false);
    if (useClosestEstimate) {
      useClosestEstimate();
    }else{
      useBestEstimate();
    }
  }

  public void useClosestEstimate(){
    var estimates = vision.estimates;
    for(var i = 0; i < estimates.size(); i++){
      var estimate = estimates.get(i);
      var estimator = vision.estimators.get(i);

      if(!Vision.isEstimatePresent(estimate)){
        continue;
      }

      // var closestEstimate = vision.closestEstimateFinder.getClosestEstimate(estimator, estimate);
      // fuseEstimate(closestEstimate.get());
    }
  }

  public void useBestEstimate(){
    var estimates = vision.estimates;
    for(var i = 0; i < estimates.size(); i++){
      var estimate = estimates.get(i);

      if(!Vision.isEstimatePresent(estimate)){
        continue;
      }

      fuseEstimate(estimate.get());
    }
  }

  public void fuseEstimate(EstimatedRobotPose visionResult) {
    var poseEstimate = visionResult.estimatedPose.toPose2d();
    var timestamp = visionResult.timestampSeconds;

    //Vary St Devs based on Chassis Spd
    boolean shouldVaryStDev = SmartDashboard.getBoolean("Vision/Vary StDev", false);
    if (shouldVaryStDev) {
      double visionStDev = 0.9;
      var vels = this.vision.swerve.getChassisSpeeds();
      var spds = Math.hypot(vels.vxMetersPerSecond, vels.vyMetersPerSecond);
      if (spds > 1.5) {
        visionStDev = 3;
      }
      var stDevs = VecBuilder.fill(visionStDev, visionStDev, visionStDev);
      this.vision.swerve.fuseVisionEstimate(poseEstimate, timestamp, stDevs);
    }else{
      this.vision.swerve.fuseVisionEstimate(poseEstimate, timestamp);
    }
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
