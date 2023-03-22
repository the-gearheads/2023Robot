// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.obsolete;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.CustomProxy;

public class ComplexAlignToFeeder extends CustomProxy {
  // 4 checks: y alignment, rotation, arm position, close to pick up pose
  // if all these checks pass, run the simple alignment routine
  
  // if any of the checks fail, run the following complex routine:
  // ensure robot is away from walls (so arm does not hit anything upon movement) ->  
  // if need to rotate -> put arm in
  // set prep pose to the current pose
  // if y is mis-aligned -> set prep pose y-position
  // move to prep pose
  // if the distance between the prep pose to dest pose is too small, raise arm first
  // move to dest pose while raising the arm
  public ComplexAlignToFeeder() {
    super(()->{
      return proxy();
    });
    addRequirements();
  }

  public static Command proxy(){
    return null;
  }
}
