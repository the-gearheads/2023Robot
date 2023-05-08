// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autoalign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AUTO_ALIGN;
import frc.robot.Constants.AUTO_ALIGN.COMMUNITY;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MoreMath;

public class GridAlign extends SequentialCommandGroup {
  public GridAlign(Swerve swerve, Arm arm) {
    if (isRotated(swerve)) {
      addCommands(simpleAlign(swerve, arm));
    } else {
      addCommands(new InstantCommand());
    }
  }

  public static boolean isRotated(Swerve swerve) {
    var currentPose = swerve.getPose();

    var destRad = Units.degreesToRadians(180);
    var currentRad = currentPose.getRotation().getRadians();

    var closestDestRad = MoreMath.getClosestRad(currentRad, destRad);

    var radDist = Units.radiansToDegrees(Math.abs(closestDestRad - currentRad));

    return radDist < AUTO_ALIGN.FEEDER.ROT_THRESHOLD;
  }


  public static Command simpleAlign(Swerve swerve, Arm arm) {
    var destPose = getDestPose(swerve, arm);
    var initSpeed = calcInitSpeed(swerve.getPose(), destPose);
    var pathCommand = swerve.goTo(destPose, COMMUNITY.CONSTRAINTS, initSpeed);
    return pathCommand;
  }

  public static double calcInitSpeed(Pose2d initPose, Pose2d endPose){
    var constraints = COMMUNITY.CONSTRAINTS;
    //v^2=v0^2+2ad
    var dist = endPose.getTranslation().minus(initPose.getTranslation()).getNorm();
    var initSpeed = Math.sqrt(2*constraints.maxAcceleration*dist) * 0.25;
    initSpeed = MathUtil.clamp(initSpeed, 0, constraints.maxVelocity);
    return initSpeed;
  }

  public static Pose2d getDestPose(Swerve swerve, Arm arm) {
    /* Assuming they have a range of 1-3 */
    var desiredGridCol = getDesiredGridCol(swerve);
    var desiredArmPose = getDesiredArmPose(arm);
    Translation2d desiredTrans;

    switch (desiredArmPose) {
      case HIGH_NODE:
        desiredTrans = desiredGridCol.high;
        break;
      case MID_NODE:
        desiredTrans = desiredGridCol.mid;
        break;
      case LOW_NODE:
        desiredTrans = desiredGridCol.low;
        break;
      default:
        desiredTrans = desiredGridCol.mid;
        break;
    }

    var destPose = new Pose2d(desiredTrans, Rotation2d.fromDegrees(180));
    return destPose;
  }

  public static ArmPose getDesiredArmPose(Arm arm) {
    ArmPose[] armPoses = {ArmPose.LOW_NODE, ArmPose.MID_NODE, ArmPose.HIGH_NODE};

    var smallestDist = Double.POSITIVE_INFINITY;
    ArmPose closestArmPose = ArmPose.HIGH_NODE;

    for (var armPose : armPoses) {
      var dist = Math.abs(armPose.val - arm.getPoseGoal().position);
      if (dist < smallestDist) {
        smallestDist = dist;
        closestArmPose = armPose;
      }
    }
    return closestArmPose;
  }

  public static GridCol getDesiredGridCol(Swerve swerve) {
    var desiredGrid = getClosestGrid(swerve.getPose());
    var desiredGridCol = desiredGrid.leftCol;
    if (Controllers.driverController.getAutoLeft().getAsBoolean()) {
      desiredGridCol = desiredGrid.leftCol;
    } else if (Controllers.driverController.getAutoCenter().getAsBoolean()) {
      desiredGridCol = desiredGrid.centerCol;
    } else if (Controllers.driverController.getAutoRight().getAsBoolean()) {
      desiredGridCol = desiredGrid.rightCol;
    }

    return desiredGridCol;
  }

  public static Community getCommunity() {
    if (MoreMath.isBlue()) {
      return Community.BLUE_GRID;
    }
    return Community.RED_GRID;
  }

  public static Grid getClosestGrid(Pose2d currentPose) {
    var community = getCommunity();
    Grid[] grids = {community.leftGrid, community.centerGrid, community.rightGrid};

    var smallestDist = Double.POSITIVE_INFINITY;
    Grid closestGrid = grids[0];

    for (var grid : grids) {
      var gridY = grid.getY();
      var dist = Math.abs(gridY - currentPose.getY());
      if (dist < smallestDist) {
        smallestDist = dist;
        closestGrid = grid;
      }
    }
    return closestGrid;
  }
}
