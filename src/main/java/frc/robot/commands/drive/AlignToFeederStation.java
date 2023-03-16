// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Collections;
import java.util.Map;
import java.util.function.Supplier;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MoreMath;
import frc.robot.Constants.AUTO_ALIGN;

public class AlignToFeederStation extends ProxyCommand {

  public AlignToFeederStation(Swerve swerve, Arm arm) {
    super(() -> {
      if (isFarAway(swerve)) {
        return new InstantCommand();
      }

      return rotateThenRaiseWhileMove(swerve, arm);
    });
  }

  private static boolean isFarAway(Swerve swerve) {
    var startPose = swerve.getPose();
    var midPose = AUTO_ALIGN.FEEDER_MID_POSE;

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      midPose = new Pose2d(midPose.getX(), Constants.FIELD_CONSTANTS.WIDTH - midPose.getY(), midPose.getRotation());
    }

    if (midPose.getTranslation().getDistance(startPose.getTranslation()) > 4.5) {
      return true;
    } else {
      return false;
    }
  }

  public static Command rotateThenRaiseWhileMove(Swerve swerve, Arm arm) {
    Supplier<Command> lambda = () -> {
      var constraints = new PathConstraints(2, 1);

      var startPose = swerve.getPose();
      var midPose = AUTO_ALIGN.FEEDER_MID_POSE;
      var endPose = AUTO_ALIGN.FEEDER_END_POSE;

      midPose = MoreMath.transformByAlliance(midPose);
      endPose = MoreMath.transformByAlliance(endPose);

      var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
      var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

      var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
      var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation(), 2);

      var endHeading = midHeading;
      var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

      var traj = PathPlanner.generatePath(constraints, startPoint, midPoint, endPoint);
      // traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

      return swerve.silentFollowTrajectoryCommand(traj, false, true);
    };

    return new rotateTo(swerve, Rotation2d.fromDegrees(180))
        .andThen(new ProxyCommand(lambda).alongWith(new SetArmPose(arm, ArmPose.FEEDER_STATION)));
  }

  public static Command raiseWhileMoveSlowerOption(Swerve swerve, Arm arm) {
    var startPose = swerve.getPose();
    var midPose = Constants.AUTO_ALIGN.FEEDER_MID_POSE;
    var endPose = Constants.AUTO_ALIGN.FEEDER_END_POSE;

    var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

    var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
    var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation());

    var endHeading = midHeading;
    var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

    var slowerConstraints = new PathConstraints(0.5, 0.25);

    var traj1 = PathPlanner.generatePath(AUTO_ALIGN.FEEDER_CONSTRAINTS, startPoint, midPoint);
    var traj2 = PathPlanner.generatePath(slowerConstraints, midPoint, endPoint);

    traj1 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj1, DriverStation.getAlliance());
    traj2 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj2, DriverStation.getAlliance());

    return swerve.followTrajectoryCommand(traj1, false, false).andThen(
        new SetArmPose(arm, ArmPose.FEEDER_STATION).alongWith(swerve.followTrajectoryCommand(traj2, false, true)));
  }

  public static Command raiseAndMoveOption(Swerve swerve, Arm arm) {
    var constraints = new PathConstraints(2, 1);
    var eventMarkers = Collections.singletonList(new EventMarker(Collections.singletonList("raise arm"), 1));

    var startPose = swerve.getPose();
    var midPose = AUTO_ALIGN.FEEDER_MID_POSE;
    var endPose = AUTO_ALIGN.FEEDER_END_POSE;

    var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

    var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
    var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation(), 0.5);

    var endHeading = midHeading;
    var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

    var traj = PathPlanner.generatePath(constraints, eventMarkers, startPoint, midPoint, endPoint);
    traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

    Map<String, Command> eventMap = Collections.singletonMap("raise arm", new SetArmPose(arm, ArmPose.FEEDER_STATION));

    return new FollowPathWithEvents(swerve.followTrajectoryCommand(traj, false, true), eventMarkers, eventMap);
  }

  public static Command raiseThenMoveOption(Swerve swerve, Arm arm) {
    var startPose = swerve.getPose();
    if (Math.abs(startPose.getRotation().getDegrees()) > 150) {
      return raiseWhileMovingSuboption(swerve, arm);
    } else {
      return raiseThenMoveSuboption(swerve, arm);
    }
  }

  public static Command raiseWhileMovingSuboption(Swerve swerve, Arm arm) {
    var startPose = swerve.getPose();
    var midPose = Constants.AUTO_ALIGN.FEEDER_MID_POSE;
    var endPose = Constants.AUTO_ALIGN.FEEDER_END_POSE;

    var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

    var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
    var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation());

    var endHeading = midHeading;
    var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

    var traj = PathPlanner.generatePath(AUTO_ALIGN.FEEDER_CONSTRAINTS, startPoint, midPoint, endPoint);

    traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

    return swerve.followTrajectoryCommand(traj, false, true).alongWith(new SetArmPose(arm, ArmPose.FEEDER_STATION));
  }

  public static Command raiseThenMoveSuboption(Swerve swerve, Arm arm) {
    var startPose = swerve.getPose();
    var midPose = Constants.AUTO_ALIGN.FEEDER_MID_POSE;
    var endPose = Constants.AUTO_ALIGN.FEEDER_END_POSE;

    var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

    var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
    var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation());

    var endHeading = midHeading;
    var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

    var traj1 = PathPlanner.generatePath(AUTO_ALIGN.FEEDER_CONSTRAINTS, startPoint, midPoint);
    var traj2 = PathPlanner.generatePath(AUTO_ALIGN.FEEDER_CONSTRAINTS, midPoint, endPoint);

    traj1 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj1, DriverStation.getAlliance());
    traj2 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj2, DriverStation.getAlliance());

    return swerve.followTrajectoryCommand(traj1, false, true).andThen(new SetArmPose(arm, ArmPose.FEEDER_STATION))
        .andThen(swerve.followTrajectoryCommand(traj2, false, true));
  }

}
