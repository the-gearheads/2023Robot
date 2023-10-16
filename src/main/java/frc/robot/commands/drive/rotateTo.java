// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MoreMath;

public class rotateTo extends Command {
  private Swerve swerve;
  private PIDController rotPIDCnt = new PIDController(5d, 0d, 0d);
  private Rotation2d rotGoal;
  private double optimizedRotGoal;
  private int satisfactionNumber;


  /** Creates a new rotateTo. */
  public rotateTo(Swerve swerve, Rotation2d rot) {
    this.swerve = swerve;
    this.rotGoal = rot;

    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.satisfactionNumber = 0;
    var ctsGyroAngle = swerve.getCtsGyroRotWithOffset().getRadians();
    this.optimizedRotGoal = MoreMath.getClosestRad(ctsGyroAngle, rotGoal.getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var ctsGyroAngle = swerve.getCtsGyroRotWithOffset().getRadians();

    var rotSpd = rotPIDCnt.calculate(ctsGyroAngle, optimizedRotGoal);
    rotSpd = MathUtil.clamp(rotSpd, -3.5, 3.5);

    swerve.driveFieldRelative(new ChassisSpeeds(0, 0, rotSpd));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var ctsGyroAngle = swerve.getCtsGyroRotWithOffset().getRadians();

    if (Math.abs(ctsGyroAngle - optimizedRotGoal) < 0.05) {
      satisfactionNumber++;
    } else {
      satisfactionNumber = 0;
    }

    return satisfactionNumber > 5;
  }
}
