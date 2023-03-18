// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.obsolete;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.Constants.AUTO_ALIGN;
import frc.robot.Constants.AUTO_ALIGN.GRID;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.Nodes.NodeX;
import frc.robot.commands.drive.Nodes.NodeY;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.CustomProxy;
import frc.robot.util.MoreMath;

public class AlignToGrid extends CustomProxy {
  /** Creates a new AlignToGrid. */
  public static PathConstraints constraints = new PathConstraints(2, 1);

  public AlignToGrid(Swerve swerve, Arm arm) {
    super(() -> {
      return proxy(swerve, arm);
    });
  }

  public static Command proxy(Swerve swerve, Arm arm){
    if(isArmRaised(arm) && isRotated(swerve) && isClose(swerve)){
      return simpleAlign(swerve, arm);
    }else{
      return new InstantCommand();
    }
  }

  private static Command simpleAlign(Swerve swerve, Arm arm) {
    var destPose = getDesiredNodePose();
    var pathCommand = swerve.goTo(destPose, GRID.CONSTRAINTS);

    var destArmPose = getDesiredArmPose();
    var armCommand = new SetArmPose(arm, destArmPose);

    return pathCommand.alongWith(armCommand);
  }

  private static boolean isArmRaised(Arm arm){
    var currentPose = arm.getPose();
    var destPose = getDesiredArmPose().val;

    var armDist = Math.abs(currentPose - destPose);

    return armDist < GRID.ARM_THRESHOLD;
  }

  private static boolean isRotated(Swerve swerve){
    var currentPose = swerve.getPose();
    var destPose = getDesiredNodePose();

    destPose = MoreMath.transformByAlliance(destPose);

    var destRad = destPose.getRotation().getRadians();
    var currentRad = currentPose.getRotation().getRadians();

    var closestDestRad = MoreMath.getClosestRad(currentRad, destRad);

    var radDist = Units.radiansToDegrees(Math.abs(closestDestRad - currentRad));

    return radDist < GRID.ROT_THRESHOLD;
  }

  private static boolean isClose(Swerve swerve) {
    var currentPose = swerve.getPose();
    var destPose = getDesiredNodePose();

    destPose = MoreMath.transformByAlliance(destPose);

    return destPose.getTranslation().getDistance(currentPose.getTranslation()) < GRID.DIST_THRESHOLD;
  }
  // public static Command proxy(Swerve swerve, Arm arm) {
  //   var nodePose = getDesiredNodePose();
  //   var currentPose = swerve.getPose();

  //   if (onChargingStation(currentPose) || inOpponentLoadingZone(currentPose) || farAway(currentPose)) {
  //     return new InstantCommand();
  //   }

  //   var startLeaf = new PointLeaf(swerve.getPose());
  //   var nodeLeaf = PointLeaf.createTree(nodePose);

  //   var leavesTraj = new ArrayList<PointLeaf>();
  //   leavesTraj.add(0, nodeLeaf);
  //   leavesTraj.add(0, nodeLeaf.getClosestChild(startLeaf));

  //   while (leavesTraj.get(0) != startLeaf) {
  //     var firstLeaf = leavesTraj.get(0);
  //     var closestChild = firstLeaf.getClosestChild(startLeaf);

  //     if (closestChild == null) {
  //       leavesTraj.add(0, startLeaf);
  //       continue;
  //     }

  //     var startLeafDist = firstLeaf.getDistance(startLeaf);
  //     var closestChildDist = firstLeaf.getDistance(closestChild);

  //     if (startLeafDist < closestChildDist) {
  //       leavesTraj.add(0, startLeaf);
  //     } else {
  //       leavesTraj.add(0, closestChild);
  //     }
  //   }

  //   var paths = leavesTrajToPaths(leavesTraj);

  //   var command = pathsToCommand(swerve, arm, paths, nodePose);
  //   return command;
  // }

  public static Pose2d getDesiredNodePose() {
    /* Assuming they have a range of 1-3 */
    var chosenGrid = Controllers.alignController.getChosenGrid();
    var chosenY = Controllers.alignController.getChosenY();
    var chosenX = Controllers.alignController.getChosenX();

    chosenGrid--;

    var yIndex = chosenGrid * 3 + chosenY;
    var yVal = NodeY.getByIndex(yIndex).getY();

    var xIndex = chosenX;
    var xVal = NodeX.getByIndex(xIndex).x;

    return new Pose2d(xVal, yVal, Rotation2d.fromDegrees(180));
  }

  public static ArmPose getDesiredArmPose(){
    var chosenX = Controllers.alignController.getChosenX();
    if(chosenX == 1){
      return ArmPose.LOW_NODE;
    }else if(chosenX == 2){
      return ArmPose.MID_NODE;
    }else{
      return ArmPose.HIGH_NODE;
    }
  }

  // public static Command pathsToCommand(Swerve swerve, Arm arm, ArrayList<PathPlannerTrajectory> paths, Pose2d endPose) {
  //   Command command = new InstantCommand();
  //   for (var i = 0; i < paths.size() - 1; i++) {
  //     command = command.andThen(swerve.followTrajectoryCommand(paths.get(i), false, true));
  //   }
  //   command = command.andThen(swerve.followTrajectoryCommand(paths.get(paths.size() - 1), false, true)
  //       .alongWith(new SetArmPose(arm, getDesiredArmPose())));

  //   return command;
  // }

  // public static ArrayList<PathPlannerTrajectory> leavesTrajToPaths(ArrayList<PointLeaf> leavesTraj) {
  //   var paths = new ArrayList<PathPlannerTrajectory>();
  //   for (int i = 1; i < leavesTraj.size(); i++) {
  //     var lastPose = leavesTraj.get(i - 1).pose;
  //     var currentPose = leavesTraj.get(i).pose;

  //     var heading = currentPose.getTranslation().minus(lastPose.getTranslation()).getAngle();

  //     var lastPoint = new PathPoint(lastPose.getTranslation(), heading, lastPose.getRotation());
  //     var currentPoint = new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation());
  //     var path = PathPlanner.generatePath(constraints, lastPoint, currentPoint);
  //     // path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
  //     paths.add(path);
  //   }
  //   return paths;
  // }

  // public static List<PathPoint> trajLeavesToPoints(ArrayList<PointLeaf> leavesTraj) {
  //   var pointsTraj = new ArrayList<PathPoint>();

  //   for (var i = 0; i < leavesTraj.size(); i++) {
  //     var leaf = leavesTraj.get(i);
  //     var heading = getHeading(i, leavesTraj, pointsTraj);
  //     var point = new PathPoint(leaf.pose.getTranslation(), heading, leaf.pose.getRotation());
  //     pointsTraj.add(point);
  //   }

  //   return pointsTraj;
  // }

  // public static Rotation2d getHeading(int index, ArrayList<PointLeaf> leavesTraj, ArrayList<PathPoint> pointsTraj) {
  //   var prevPoint = index - 1 < 0 ? null : pointsTraj.get(index - 1);
  //   var currentLeaf = leavesTraj.get(index);
  //   var nextLeaf = index + 1 >= leavesTraj.size() ? null : leavesTraj.get(index + 1);

  //   if (nextLeaf == null) {
  //     return currentLeaf.pose.getTranslation().minus(prevPoint.position).getAngle();
  //   } else if (prevPoint == null) {
  //     return nextLeaf.pose.getTranslation().minus(currentLeaf.pose.getTranslation()).getAngle();
  //   } else {
  //     return nextLeaf.pose.getTranslation().minus(prevPoint.position).getAngle();
  //   }
  // }

  // public static boolean onChargingStation(Pose2d pose) {
  //   var xCheck = pose.getX() < 4.45 && pose.getX() > 3.342;
  //   var yCheck = pose.getY() < 3.7 && pose.getX() > 1.875;
  //   if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
  //     yCheck = pose.getY() > (Constants.FIELD_CONSTANTS.WIDTH - 3.7)
  //         && pose.getY() < (Constants.FIELD_CONSTANTS.WIDTH - 1.875);
  //   }

  //   return xCheck && yCheck;
  // }

  // public static boolean inOpponentLoadingZone(Pose2d pose) {
  //   var xCheck = pose.getX() < 6.309;
  //   var yCheck = pose.getY() > 5.877;

  //   if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
  //     yCheck = pose.getY() < (Constants.FIELD_CONSTANTS.WIDTH - 5.877);
  //   }
  //   return xCheck && yCheck;
  // }

  // public static boolean farAway(Pose2d pose) {
  //   return pose.getX() > 7.233;
  // }
}
