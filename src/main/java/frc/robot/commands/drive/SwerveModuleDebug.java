// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;

public class SwerveModuleDebug extends CommandBase {
  private Swerve swerve;

  /** Creates a new SwerveModuleDebug. */
  public SwerveModuleDebug(Swerve swerve) {
    addRequirements(swerve);
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("SW Debug/SW Angle Setpoint", 0);
    SmartDashboard.putNumber("SW Debug/SW kP", 0);
    SmartDashboard.putNumber("SW Debug/SW kI", 0);
    SmartDashboard.putNumber("SW Debug/SW kD", 0);


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint = SmartDashboard.getNumber("SW Debug/Left Mod Angle Setpoint", 0);
    swerve.modules[1].setState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(setpoint)));

    double kP = SmartDashboard.getNumber("SW Debug/SW kP", 0);
    double kI = SmartDashboard.getNumber("SW Debug/SW kI", 0);
    double kD = SmartDashboard.getNumber("SW Debug/SW kD", 0);
    swerve.modules[1].setPIDConstants(kP, kI, kD, setpoint);
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
