// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristState.WristStateType;

public class AltWristControl extends CommandBase {
  private Wrist wrist;

  /** Creates a new SetWristAlternatePose. */
  public AltWristControl(Wrist wrist) {
    this.wrist = wrist;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Alt wrist control turned on?", "ON");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setGoalByType(WristStateType.ALT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Alt wrist control turned on?", "OFF");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
