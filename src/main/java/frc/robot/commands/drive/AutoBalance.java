// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AUTON;
import frc.robot.subsystems.drive.Swerve;

public class AutoBalance extends CommandBase {
  // assumes already on chargin station.
  private Swerve swerve;
  private PIDController balancePid = AUTON.AUTO_BALANCE_PID;

  /** Creates a new AutoBalance. */
  public AutoBalance(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    SmartDashboard.putData("AutoBalancePID", balancePid);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePid.reset();
    balancePid.setTolerance(1.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = swerve.getPitch();
    double roll = swerve.getRoll();

    var vx = balancePid.calculate(roll, 0);

    swerve.drive(new ChassisSpeeds(vx, 0, 0));

  }

  // Called once the commak nd ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
