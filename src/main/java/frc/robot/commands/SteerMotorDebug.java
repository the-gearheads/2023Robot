// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class SteerMotorDebug extends CommandBase {//Currently only affects FL Steer Motor
  private SwerveSubsystem swerveSubsystem;
  /** Creates a new SteerMotorDebug. */
  public SteerMotorDebug(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem=swerveSubsystem;
    addRequirements(swerveSubsystem);
    double currentAngle=swerveSubsystem.lastInputs[0].currentAngle;
    SmartDashboard.putNumber("Desired Steer Motor Angle", currentAngle);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentAngle=swerveSubsystem.lastInputs[0].currentAngle;
    SmartDashboard.putNumber("Desired Steer Motor Angle", currentAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle=swerveSubsystem.lastInputs[0].currentAngle;
    SmartDashboard.putNumber("Current Steer Motor Angle", currentAngle);

    double desiredAngle=SmartDashboard.getNumber("Desired Steer Motor Angle", currentAngle);
    swerveSubsystem.setStates(new SwerveModuleState[] {new SwerveModuleState(0, Rotation2d.fromDegrees(desiredAngle))});
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
