// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;

public class AlignToGrid extends ProxyCommand {
  /** Creates a new AlignToGrid. */
  public static PathConstraints constraints = new PathConstraints(2, 1);

  public AlignToGrid(Swerve swerve, Arm arm, Pose2d nodePose) {
    super(() -> {
      var currentPose = swerve.getPose();

      if (onChargingStation(currentPose) || inOpponentLoadingZone(currentPose) || farAway(currentPose)) {
        return new InstantCommand(() -> {
        });
      }

      var startLeaf = new PointLeaf(swerve.getPose());
      var nodeLeaf = PointLeaf.createTree(nodePose);

      var leavesTraj = new ArrayList<PointLeaf>();
      leavesTraj.add(0, nodeLeaf);

      while (leavesTraj.get(0) != startLeaf) {
        var firstLeaf = leavesTraj.get(0);
        var closestChild = firstLeaf.getClosestChild(startLeaf);

        if (closestChild == null) {
          leavesTraj.add(0, startLeaf);
          continue;
        }

        var startLeafDist = firstLeaf.getDistance(startLeaf);
        var closestChildDist = firstLeaf.getDistance(closestChild);

        if (startLeafDist < closestChildDist) {
          leavesTraj.add(0, startLeaf);
        } else {
          leavesTraj.add(0, closestChild);
        }
      }

      var paths = trajLeavesToPaths(leavesTraj);

      var command = pathsToCommand(swerve, arm, paths, nodePose);
      return command;
    });
  }

  public static Command pathsToCommand(Swerve swerve, Arm arm, ArrayList<PathPlannerTrajectory> paths, Pose2d endPose) {
    Command command = new InstantCommand();
    if (paths.size() < 2) {
      command =
          swerve.followTrajectoryCommand(paths.get(0), false, true).andThen(new SetArmPose(arm, ArmPose.HIGH_NODE));
    } else {
      for (var i = 0; i < paths.size() - 1; i++) {
        command = command.andThen(swerve.followTrajectoryCommand(paths.get(i), false, true));
      }
      command = command
          .andThen(new ParallelCommandGroup(swerve.followTrajectoryCommand(paths.get(paths.size() - 1), false, true),
              new SetArmPose(arm, ArmPose.HIGH_NODE)));
    }
    var lastPose = new Pose2d(1.9, endPose.getY(), endPose.getRotation());
    var heading = lastPose.getTranslation().minus(endPose.getTranslation()).getAngle();
    var lastPoint = new PathPoint(lastPose.getTranslation(), heading, lastPose.getRotation());
    var endPoint = new PathPoint(endPose.getTranslation(), heading, endPose.getRotation());
    var path = PathPlanner.generatePath(constraints, endPoint, lastPoint);
    command = command.andThen(swerve.followTrajectoryCommand(path, false, false));

    return command;
  }

  public static ArrayList<PathPlannerTrajectory> trajLeavesToPaths(ArrayList<PointLeaf> leavesTraj) {
    var paths = new ArrayList<PathPlannerTrajectory>();
    for (int i = 1; i < leavesTraj.size(); i++) {
      var lastPose = leavesTraj.get(i - 1).pose;
      var currentPose = leavesTraj.get(i).pose;

      var heading = currentPose.getTranslation().minus(lastPose.getTranslation()).getAngle();

      var lastPoint = new PathPoint(lastPose.getTranslation(), heading, lastPose.getRotation());
      var currentPoint = new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation());
      paths.add(PathPlanner.generatePath(constraints, lastPoint, currentPoint));
    }
    return paths;
  }

  public static List<PathPoint> trajLeavesToPoints(ArrayList<PointLeaf> leavesTraj) {
    var pointsTraj = new ArrayList<PathPoint>();

    for (var i = 0; i < leavesTraj.size(); i++) {
      var leaf = leavesTraj.get(i);
      var heading = getHeading(i, leavesTraj, pointsTraj);
      var point = new PathPoint(leaf.pose.getTranslation(), heading, leaf.pose.getRotation());
      pointsTraj.add(point);
    }

    return pointsTraj;
  }

  public static Rotation2d getHeading(int index, ArrayList<PointLeaf> leavesTraj, ArrayList<PathPoint> pointsTraj) {
    var prevPoint = index - 1 < 0 ? null : pointsTraj.get(index - 1);
    var currentLeaf = leavesTraj.get(index);
    var nextLeaf = index + 1 >= leavesTraj.size() ? null : leavesTraj.get(index + 1);

    if (nextLeaf == null) {
      return currentLeaf.pose.getTranslation().minus(prevPoint.position).getAngle();
    } else if (prevPoint == null) {
      return nextLeaf.pose.getTranslation().minus(currentLeaf.pose.getTranslation()).getAngle();
    } else {
      return nextLeaf.pose.getTranslation().minus(prevPoint.position).getAngle();
    }
  }

  public static boolean onChargingStation(Pose2d pose) {
    var xCheck = pose.getX() < 4.45 && pose.getX() > 3.342;
    var yCheck = pose.getY() < 3.7 && pose.getX() > 1.875;

    return xCheck && yCheck;
  }

  public static boolean inOpponentLoadingZone(Pose2d pose) {
    var xCheck = pose.getX() < 6.309;
    var yCheck = pose.getY() > 5.877;

    return xCheck && yCheck;
  }

  public static boolean farAway(Pose2d pose) {
    return pose.getX() > 7.233;
  }
}
