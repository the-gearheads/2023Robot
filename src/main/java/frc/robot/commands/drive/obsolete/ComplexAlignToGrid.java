// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.obsolete;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.CustomProxy;

public class ComplexAlignToGrid extends CustomProxy {
  // 4 checks: y alignment, rotation, arm position, close to pick up pose
  // if all these checks pass, run the simple alignment routine

  // if any of the checks fail, run the following complex routine:
  // ensure robot is away from walls (so arm does not hit anything upon movement) ->  
  // if need to rotate -> put arm in
  // calculate trajectory
  // raise arm during last movement
  public ComplexAlignToGrid() {
    super(() -> {
      return proxy();
    });
    addRequirements();
  }

  public static Command proxy() {
    return null;
  }
}
