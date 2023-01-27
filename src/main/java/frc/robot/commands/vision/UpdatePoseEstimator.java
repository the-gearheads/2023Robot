// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class UpdatePoseEstimator extends CommandBase {
  private Vision vision;
  private SwerveSubsystem swerveSubsystem;

  /** Creates a new UpdatePoseEstimator. */
  public UpdatePoseEstimator(Vision vision, SwerveSubsystem swerveSubsystem) {
    this.vision=vision;
    this.swerveSubsystem=swerveSubsystem;
    addRequirements(vision);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isAprilTagInView = vision.isConnected() && vision.hasTargets();
    if (isAprilTagInView) {
      Optional<EstimatedRobotPose> visionResult = vision.getEstimatedGlobalPos(swerveSubsystem.getPose());
      if (Vision.isEstimatedRobotPosPresent(visionResult)) {
        Pose2d estimatedRobotPos = visionResult.get().estimatedPose.toPose2d();
        double timestamp = visionResult.get().timestampSeconds;
        swerveSubsystem.updateVisionMeasurement(estimatedRobotPos, timestamp);
      }
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
