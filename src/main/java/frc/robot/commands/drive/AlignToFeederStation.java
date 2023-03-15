// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;

public class AlignToFeederStation extends ProxyCommand {
  /** Creates a new PickUp. */
  public AlignToFeederStation(Swerve swerve, Arm arm) {
    super(() -> {
      var startPose = swerve.getPose();
      var midPose = new Pose2d(14.5, 7.5, Rotation2d.fromDegrees(180));
      var endPose = new Pose2d(15.5, 7.5, Rotation2d.fromDegrees(180));

      //please transform this (only works with blue)
      if (midPose.getTranslation().getDistance(startPose.getTranslation()) > 4.5) {
        return new InstantCommand();
      }

      if (Math.abs(startPose.getRotation().getDegrees()) > 150) {
        return raiseWhileMoving(swerve, arm);
      } else {
        return raiseThenMove(swerve, arm);
      }
    });
  }

  public static Command raiseWhileMoving(Swerve swerve, Arm arm) {
    var startPose = swerve.getPose();
    var midPose = new Pose2d(14.5, 7.5, Rotation2d.fromDegrees(180));
    var endPose = new Pose2d(15.5, 7.5, Rotation2d.fromDegrees(180));

    var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

    var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
    var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation());

    var endHeading = midHeading;
    var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

    var constraints1 = new PathConstraints(2, 1);

    var traj = PathPlanner.generatePath(constraints1, startPoint, midPoint, endPoint);

    traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

    return swerve.followTrajectoryCommand(traj, false, true).alongWith(new SetArmPose(arm, ArmPose.FEEDER_STATION));
  }

  public static Command raiseThenMove(Swerve swerve, Arm arm) {
    var startPose = swerve.getPose();
    var midPose = new Pose2d(14.5, 7.5, Rotation2d.fromDegrees(180));
    var endPose = new Pose2d(15.5, 7.5, Rotation2d.fromDegrees(180));

    var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

    var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
    var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation());

    var endHeading = midHeading;
    var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

    var constraints1 = new PathConstraints(2, 1);
    var constraints2 = new PathConstraints(2, 1);

    var traj1 = PathPlanner.generatePath(constraints1, startPoint, midPoint);
    var traj2 = PathPlanner.generatePath(constraints2, midPoint, endPoint);

    traj1 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj1, DriverStation.getAlliance());
    traj2 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj2, DriverStation.getAlliance());

    return swerve.followTrajectoryCommand(traj1, false, true).andThen(new SetArmPose(arm, ArmPose.FEEDER_STATION))
        .andThen(swerve.followTrajectoryCommand(traj2, false, true));
  }

  // public AlignToFeederStation(Swerve swerve, Arm arm) {
  //   super(()->{
  //     var constraints = new PathConstraints(2, 1);
  //     var eventMarkers =Collections.singletonList(
  //       new EventMarker(
  //         Collections.singletonList( "raise arm" ), 1)
  //       );

  //     var startPose = swerve.getPose();
  //     var midPose = new Pose2d(10.9, 7.5, Rotation2d.fromDegrees(180));
  //     var endPose = new Pose2d(15.5, 7.5, Rotation2d.fromDegrees(180));

  //     var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
  //     var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

  //     var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
  //     var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation(), 0.5);

  //     var endHeading = midHeading;
  //     var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

  //     var traj = PathPlanner.generatePath(constraints, eventMarkers, startPoint, midPoint, endPoint);
  //     traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

  //     Map<String, Command> eventMap = Collections.singletonMap("raise arm",new SetArmPose(arm, ArmPose.FEEDER_STATION));

  //     return new FollowPathWithEvents(swerve.followTrajectoryCommand(traj, false, true), eventMarkers, eventMap);
  //   });
  // }
}
