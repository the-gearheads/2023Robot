// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.AUTO_ALIGN.FEEDER;
import frc.robot.Constants.AUTO_ALIGN.GRID;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MoreMath;

public class AutoAlign extends ProxyCommand {
  public AutoAlign(Swerve swerve, Arm arm) {
    super(() -> {
      return proxy(swerve, arm);
    });
  }

  public static Command proxy(Swerve swerve, Arm arm) {
    if (inCommunity(swerve.getPose())) {
      return new GridAlign(swerve, arm);
    } else if (inFeederArea(swerve.getPose())) {
      return new FeederAlign(swerve, arm);
    } else {
      return new InstantCommand();
    }

  }

  private static boolean inCommunity(Pose2d pose) {
    return pose.getX() < GRID.COMMUNITY_THRESHOLD;
  }

  private static boolean inFeederArea(Pose2d pose) {
    var xCheck = MoreMath.within(pose.getX(), FEEDER.MIN_X, FEEDER.MAX_X);
    var yCheck = MoreMath.within(pose.getY(), FEEDER.MIN_Y, FEEDER.MAX_Y);
    if (!MoreMath.isBlue()) {
      yCheck = MoreMath.within(pose.getY(), FIELD_CONSTANTS.WIDTH - FEEDER.MIN_Y, FIELD_CONSTANTS.WIDTH - FEEDER.MAX_Y);
    }
    return xCheck && yCheck;
  }
}
