// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState.WristControlType;

public class DefaultWristControl extends CommandBase {
  private Wrist wrist;

  public DefaultWristControl(Wrist wrist) {
    this.wrist = wrist;
    addRequirements(wrist);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void execute() {
    wrist.setGoalByType(WristControlType.DEFAULT);
  }
}
