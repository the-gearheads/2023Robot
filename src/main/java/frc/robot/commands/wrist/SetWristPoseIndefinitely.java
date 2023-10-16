// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;

public class SetWristPoseIndefinitely extends Command {
  private double pose;
  private Wrist wrist;
  private Supplier<Boolean> shouldSet;

  public SetWristPoseIndefinitely(Wrist wrist, double pose, Supplier<Boolean> shouldSet) {
    this.shouldSet = shouldSet;
    addRequirements(wrist);
    this.pose = pose;
    this.wrist = wrist;
  }

  public SetWristPoseIndefinitely(Wrist wrist, double pose) {
    this(wrist, pose, () -> true);
  }

  @Override
  public void execute() {
    if (!shouldSet.get())
      return;
    var wristState = WristState.getStateWithGoal(pose);
    wrist.setGoal(wristState);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
