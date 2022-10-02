// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(SwerveSubsystem subsystem) {
    swerveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.zeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpd = Controllers.activeController.getXMoveAxis() * Constants.Drivetrain.MAX_LIN_VEL;
    var ySpd = Controllers.activeController.getYMoveAxis() * Constants.Drivetrain.MAX_LIN_VEL;
    var rotSpd = Controllers.activeController.getRotateAxis() * Constants.Drivetrain.MAX_ROT_VEL;
    swerveSubsystem.driveFieldRelative(new ChassisSpeeds(xSpd, ySpd, rotSpd));
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
