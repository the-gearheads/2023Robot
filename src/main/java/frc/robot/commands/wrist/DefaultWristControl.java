// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.WristState.WristStateType;
import frc.robot.subsystems.wrist.Wrist;

public class DefaultWristControl extends CommandBase {
  private Wrist wrist;

  /** Creates a new DefaultWristControl. */
  public DefaultWristControl(Wrist wrist) {
    this.wrist = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Default wrist control turned on?", "ON");
    wrist.setControlState(WristStateType.DEFAULT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Default wrist control turned on?", "OFF");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
