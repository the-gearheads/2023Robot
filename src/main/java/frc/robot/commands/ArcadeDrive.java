// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putBoolean("ArcadeDrive/UseFieldRelative", true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.zeroEncoders();
    SmartDashboard.putNumber("F",Constants.Drivetrain.STEER_F);
    SmartDashboard.putNumber("P",Constants.Drivetrain.STEER_P);
    SmartDashboard.putNumber("I",Constants.Drivetrain.STEER_I);
    SmartDashboard.putNumber("D",Constants.Drivetrain.STEER_D);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpd = Controllers.activeController.getXMoveAxis() * Constants.Drivetrain.MAX_LIN_VEL;
    var ySpd = Controllers.activeController.getYMoveAxis() * Constants.Drivetrain.MAX_LIN_VEL;
    var rotSpd = Controllers.activeController.getRotateAxis() * Constants.Drivetrain.MAX_ROT_VEL;

    SmartDashboard.putNumber("ArcadeDrive/xSpd", xSpd);
    SmartDashboard.putNumber("ArcadeDrive/ySpd", ySpd);
    SmartDashboard.putNumber("ArcadeDrive/rot", rotSpd);

    if(SmartDashboard.getBoolean("ArcadeDrive/UseFieldRelative", true)) {
      swerveSubsystem.driveFieldRelative(new ChassisSpeeds(xSpd, ySpd, rotSpd));
    } else {
      swerveSubsystem.drive(new ChassisSpeeds(xSpd, ySpd, rotSpd));
    }

    updatePIDConstants();
  }

  private void updatePIDConstants(){
    double kF=SmartDashboard.getNumber("F",Constants.Drivetrain.STEER_F);
    double kP=SmartDashboard.getNumber("P",Constants.Drivetrain.STEER_P);
    double kI=SmartDashboard.getNumber("I",Constants.Drivetrain.STEER_I);
    double kD=SmartDashboard.getNumber("D",Constants.Drivetrain.STEER_D);
    swerveSubsystem.setPIDConstants(kF, kP, kI, kD);
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
