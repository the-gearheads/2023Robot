// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;

public class AutoBalance extends CommandBase {
  // assumes already on chargin station.
  private Swerve swerve;

  /** Creates a new AutoBalance. */
  public AutoBalance(Swerve swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = swerve.getPitch();
    double roll = swerve.getRoll();
    double vx = 0;
    double vy = 0;
    if (pitch > 5) {
      vx = -0.5;
    } else if (pitch < -5) {
      vx = 0.5;
    }
    if (roll > 5) {
      vy = -0.5;
    } else if (roll < -5) {
      vy = 0.5;
    }

    swerve.drive(new ChassisSpeeds(vx, vy, 0));

  }

  // Called once the commak nd ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double pitch = swerve.getPitch();
    double roll = swerve.getRoll();
    return Math.abs(pitch) < 5 && Math.abs(roll) < 5;
  }
}
