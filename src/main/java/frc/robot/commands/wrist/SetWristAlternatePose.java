// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import java.util.function.Function;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ARM;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristState.WristStateType;

public class SetWristAlternatePose extends CommandBase {
  private Arm arm;
  private Wrist wrist;

  /** Creates a new SetWristAlternatePose. */
  public SetWristAlternatePose(Wrist wrist, Arm arm) {
    this.wrist = wrist;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double armPos = arm.getPosition();
    for (WristState wristState : WristState.values()) {
      if (wristState.inRange(armPos) && wristState.type==WristStateType.ALT) {
        wrist.setGoal(wristState.getWristGoal(armPos));
        return;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
