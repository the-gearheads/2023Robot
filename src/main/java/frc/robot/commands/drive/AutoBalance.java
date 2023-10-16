// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AUTON;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.drive.Swerve;

public class AutoBalance extends Command {
  // assumes already on charging station.
  private Swerve swerve;
  private Grabber grabber;
  private PIDController balancePid = AUTON.AUTO_BALANCE_PID;

  /** Creates a new AutoBalance. */
  public AutoBalance(Swerve swerve, Grabber grabber) {
    this.swerve = swerve;
    this.grabber = grabber;
    addRequirements(swerve);
    SmartDashboard.putData("AutoBalancePID", balancePid);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePid.reset();
    balancePid.setTolerance(1.5);
    grabber.disableCompressor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = swerve.getRoll();
    var vx = balancePid.calculate(roll, -2.6);

    swerve.drive(new ChassisSpeeds(vx, 0, 0));

    SmartDashboard.putNumber("autobalance/roll", roll);
    SmartDashboard.putNumber("autobalance/vx", vx);
  }

  // Called once the commak nd ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setX();
    swerve.drive(new ChassisSpeeds());
    grabber.enableCompressor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
