// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;

public class ManualWristControl extends CommandBase {
  private Wrist wrist;
  private WristState state;

  /** Creates a new ManualWristControl. */
  public ManualWristControl(Wrist wrist, WristState state) {
    this.wrist = wrist;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setGoal(state);
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
