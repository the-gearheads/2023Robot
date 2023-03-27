// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.obsolete;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AUTO_ALIGN.COMMUNITY;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.obsolete.Nodes.NodeX;
import frc.robot.commands.drive.obsolete.Nodes.NodeY;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MoreMath;

public class GridAlignWithNodes extends SequentialCommandGroup {
  public GridAlignWithNodes(Swerve swerve, Arm arm) {
    addCommands(simpleAlign(swerve, arm));
  }


  public static Command simpleAlign(Swerve swerve, Arm arm) {
    var destPose = getDestPose(swerve, arm);
    // var desiredArmPose = getDesiredArmPose(arm);

    var pathCommand = swerve.goTo(destPose, COMMUNITY.CONSTRAINTS);
    // var armCommand = new SetArmPose(arm, desiredArmPose);
    return pathCommand;
  }

  public static Pose2d getDestPose(Swerve swerve, Arm arm) {
    /* Assuming they have a range of 1-3 */
    var x = getDesiredNodeX(arm).x;
    var y = getDesiredNodeY(swerve).getY();

    return new Pose2d(x, y, Rotation2d.fromDegrees(180));
  }

  public static NodeX getDesiredNodeX(Arm arm) {
    var desiredArmPose = getDesiredArmPose(arm);
    switch (desiredArmPose) {
      case HIGH_NODE:
        return NodeX.HIGH;
      case MID_NODE:
        return NodeX.MID;
      case LOW_NODE:
        return NodeX.LOW;
      default:
        return NodeX.HIGH;
    }
  }

  public static ArmPose getDesiredArmPose(Arm arm) {
    ArmPose[] armPoses = {ArmPose.LOW_NODE, ArmPose.MID_NODE, ArmPose.HIGH_NODE};

    var smallestDist = Double.POSITIVE_INFINITY;
    ArmPose closestArmPose = ArmPose.HIGH_NODE;

    for (var armPose : armPoses) {
      var dist = Math.abs(armPose.val - arm.getPose());
      if (dist < smallestDist) {
        smallestDist = dist;
        closestArmPose = armPose;
      }
    }
    return closestArmPose;
  }

  public static NodeY getDesiredNodeY(Swerve swerve) {
    var indexDelta = 0;
    if (Controllers.driverController.getAutoLeft().getAsBoolean()) {
      indexDelta = -1;
    } else if (Controllers.driverController.getAutoCenter().getAsBoolean()) {
      indexDelta = 0;
    } else if (Controllers.driverController.getAutoRight().getAsBoolean()) {
      indexDelta = 1;
    }

    //I hate mirrored fields
    if (!MoreMath.isBlue()) {
      indexDelta *= -1;
    }

    var closestCenterNodeIndex = getClosestCenterNode(swerve.getPose()).index;
    var nodeY = NodeY.getByIndex(closestCenterNodeIndex + indexDelta);
    return nodeY;
  }

  public static NodeY getClosestCenterNode(Pose2d currentPose) {
    NodeY[] centerNodes = {NodeY.N2, NodeY.N5, NodeY.N8};

    var smallestDist = Double.POSITIVE_INFINITY;
    NodeY closestNode = NodeY.getByIndex(1);

    for (var centerNode : centerNodes) {
      var dist = Math.abs(centerNode.getY() - currentPose.getY());
      if (dist < smallestDist) {
        smallestDist = dist;
        closestNode = centerNode;
      }
    }
    return closestNode;
  }
}
