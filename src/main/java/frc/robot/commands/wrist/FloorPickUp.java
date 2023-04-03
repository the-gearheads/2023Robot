// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class FloorPickUp extends ParallelCommandGroup {
  /** Creates a new PickUpFromGround. */
  public FloorPickUp(Arm arm, Wrist wrist) {
    super(new SetArmPose(arm, -74), new AltWristControl(wrist));
  }
}
