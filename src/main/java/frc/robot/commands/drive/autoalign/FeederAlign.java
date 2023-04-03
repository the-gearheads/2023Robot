// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autoalign;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AUTO_ALIGN.FEEDER;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MoreMath;

public class FeederAlign extends SequentialCommandGroup {
  /** Creates a new FeederAlign. */
  public FeederAlign(Swerve swerve, Arm arm) {
    if (
    // isYAligned(swerve) && 
    isArmRaised(arm) &&
    // isClose(swerve) && 
        isRotated(swerve)) {
      addCommands(simpleAlign(swerve, arm));
    } else {
      // still needs some more work
      // addCommands(prepThenGoToDest(swerve, arm));
      addCommands(new InstantCommand());
    }
  }

  private static Command simpleAlign(Swerve swerve, Arm arm) {
    var startPose = swerve.getPose();
    var endPose = getDestPose(startPose);
    var traj = MoreMath.createStraightPath(startPose, endPose, FEEDER.CONSTRAINTS);

    var trajCommand = swerve.followTrajectoryCommand(traj, false, true);
    // var armCommand = new SetArmPose(arm, ArmPose.FEEDER_STATION);

    return trajCommand;
  }

  private static Pose2d getDestPose(Pose2d pose) {
    Pose2d destPose;
    if (isLeft(pose)) {
      if (MoreMath.isBlue()) {
        destPose = FEEDER.BLUE_LEFT_DEST_POSE;
      } else {
        destPose = FEEDER.RED_LEFT_DEST_POSE;
      }
    } else {
      if (MoreMath.isBlue()) {
        destPose = FEEDER.BLUE_RIGHT_DEST_POSE;
      } else {
        destPose = FEEDER.RED_RIGHT_DEST_POSE;
      }
    }
    return destPose;
  }

  private static boolean isYAligned(Swerve swerve) {
    var currentPose = swerve.getPose();
    var destPose = getDestPose(currentPose);

    var yDist = Math.abs(destPose.getY() - currentPose.getY());

    if (yDist < FEEDER.Y_THRESHOLD) {
      return true;
    }
    return false;
  }

  private static boolean isArmRaised(Arm arm) {
    var currentArmPose = arm.getPose();
    var desiredArmPose = ArmPose.FEEDER_STATION.val;

    var armDist = Math.abs(currentArmPose - desiredArmPose);

    return armDist < FEEDER.ARM_THRESHOLD;
  }

  public static boolean isRotated(Swerve swerve) {
    var currentPose = swerve.getPose();
    var destPose = getDestPose(currentPose);

    var destRad = destPose.getRotation().getRadians();
    var currentRad = currentPose.getRotation().getRadians();

    var closestDestRad = MoreMath.getClosestRad(currentRad, destRad);

    var radDist = Units.radiansToDegrees(Math.abs(closestDestRad - currentRad));

    return radDist < FEEDER.ROT_THRESHOLD;
  }

  private static boolean isClose(Swerve swerve) {
    var currentPose = swerve.getPose();
    var destPose = getDestPose(swerve.getPose());

    return destPose.getTranslation().getDistance(currentPose.getTranslation()) < FEEDER.CLOSE_THRESHOLD;
  }

  private static boolean isLeft(Pose2d pose) {
    return (Controllers.driverController.getAutoLeft().getAsBoolean());
  }

  // private static Pose2d getPrepPose(Pose2d pose) {
  //   Pose2d prepPose;
  //   if (isLeft(pose)) {
  //     prepPose = FEEDER.LEFT_PREP_POSE;
  //   } else {
  //     prepPose = FEEDER.RIGHT_PREP_POSE;
  //   }
  //   prepPose = MoreMath.transformByAlliance(prepPose);
  //   return prepPose;
  // }

  // private static Command prepThenGoToDest(Swerve swerve, Arm arm) {

  //   var startPose = swerve.getPose();
  //   var prepPose = getPrepPose(startPose);
  //   var destPose = getDestPose(startPose);

  //   var startHeading = MoreMath.calcHeading(startPose, prepPose);
  //   var startPoint = MoreMath.createPathPoint(startPose, startHeading);

  //   var prepHeading = MoreMath.calcHeading(prepPose, destPose);
  //   var prepPoint = MoreMath.createPathPoint(prepPose, prepHeading);

  //   var destHeading = prepHeading;
  //   var destPoint = MoreMath.createPathPoint(destPose, destHeading);

  //   var startToPrepTraj = PathPlanner.generatePath(FEEDER.CONSTRAINTS, startPoint, prepPoint);
  //   var startToPrepCommand = swerve.followTrajectoryCommand(startToPrepTraj, false, true);

  //   var prepToDestTraj = PathPlanner.generatePath(FEEDER.CONSTRAINTS, prepPoint, destPoint);
  //   var prepToDestCommand = swerve.followTrajectoryCommand(prepToDestTraj, false, true);

  //   var rotateCommand = new rotateTo(swerve, Rotation2d.fromDegrees(180));

  //   var raiseArmCommand = new SetArmPose(arm, ArmPose.FEEDER_STATION);

  //   return new SequentialCommandGroup(startToPrepCommand, rotateCommand, raiseArmCommand, prepToDestCommand);
  // }
}
