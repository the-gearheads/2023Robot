// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.CamServo;
import frc.robot.subsystems.vision.Vision;

public class TrackAprilTags extends CommandBase {
  /** Creates a new TrackAprilTags. */
  private Vision vision;
  private CamServo camServo;
  @SuppressWarnings("all")
  private SwerveSubsystem swerveSubsystem;

  public TrackAprilTags(Vision vision, CamServo camServo, SwerveSubsystem swerveSubsystem) {
    this.vision=vision;
    this.camServo=camServo;
    this.swerveSubsystem=swerveSubsystem;
    addRequirements(vision);
    // Use addRequirements() here to declare subsystem dependencies.
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
      // Update swerve pos estimator
      Optional<EstimatedRobotPose> visionResult = vision.getEstimatedGlobalPos(swerveSubsystem.getPose());

      if(Vision.isEstimatedRobotPosPresent(visionResult)){
        Pose2d estimatedRobotPos = visionResult.get().estimatedPose.toPose2d();
        double timestamp = visionResult.get().timestampSeconds;
        swerveSubsystem.updateVisionMeasurement(estimatedRobotPos, timestamp);

        //follow april tag
        double aprilTagAngleInFrame = estimatedRobotPos.getRotation().getDegrees();
        double desiredServoAngle = camServo.getCurrentAngle() - aprilTagAngleInFrame;
        desiredServoAngle = MathUtil.clamp(desiredServoAngle, 0, 180);
      }
      
    }else{
      // wander();
    }
  }

  public void wander(){
    if (Math.abs(camServo.getGoal() - camServo.getCurrentAngle()) < 1e-1){
      double nextCommmandedAngle = MathUtil.applyDeadband(camServo.getGoal() - 180, 1e-1) == 0 ? 0 : 180;
      camServo.setGoal(nextCommmandedAngle);
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
