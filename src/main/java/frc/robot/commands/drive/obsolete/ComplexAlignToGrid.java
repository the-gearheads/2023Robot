// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.obsolete;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.CustomProxy;

public class ComplexAlignToGrid extends CustomProxy {

  public ComplexAlignToGrid() {
    super(()->{
      return proxy();
    });
    addRequirements();
  }

  public static Command proxy(){
    return null;
  }
}
