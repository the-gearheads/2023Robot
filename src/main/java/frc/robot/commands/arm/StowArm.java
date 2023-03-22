// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.wrist.ManualWristControl;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;

public class StowArm extends SequentialCommandGroup {
  /** Creates a new StowArm. */
  public StowArm(Arm arm, Wrist wrist) {
    super(new SetArmPose(arm, ArmPose.INSIDE_ROBOT),
        new ManualWristControl(wrist, WristState.RIGHT));
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
