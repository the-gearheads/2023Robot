// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.sql.Driver;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.CustomProxy;
import frc.robot.util.MoreMath;
import frc.robot.Constants.AUTO_ALIGN;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.AUTO_ALIGN.FEEDER;

public class AlignToFeederStation extends CustomProxy {

  public AlignToFeederStation(Swerve swerve, Arm arm) {
    super(()->{
      return proxy(swerve, arm);
    }, swerve, arm);
  }

  private static Command proxy(Swerve swerve, Arm arm){
    if(!inFeederArea(swerve)){
      return new InstantCommand();
    }
    
    if(isYAligned(swerve)
    && isArmRaised(arm)
    && isClose(swerve)
    && isRotated(swerve)){
      return goToDest(swerve, arm);
    }else{
      return prepThenGoToDest(swerve, arm);
    }
  }

  private static Command prepThenGoToDest(Swerve swerve, Arm arm) {

      var startPose = swerve.getPose();
      var prepPose = FEEDER.PREP_POSE;
      var destPose = FEEDER.DEST_POSE;

      prepPose = MoreMath.transformByAlliance(prepPose);
      destPose = MoreMath.transformByAlliance(destPose);

      var startHeading = MoreMath.calcHeading(startPose, prepPose);
      var startPoint = MoreMath.createPathPoint(startPose, startHeading);

      var prepHeading = MoreMath.calcHeading(prepPose, destPose);
      var prepPoint = MoreMath.createPathPoint(prepPose, prepHeading);

      var destHeading = prepHeading;
      var destPoint = MoreMath.createPathPoint(destPose, destHeading);

      var startToPrepTraj = PathPlanner.generatePath(FEEDER.CONSTRAINTS, startPoint, prepPoint);
      var startToPrepCommand = swerve.followTrajectoryCommand(startToPrepTraj, false, true);

      var prepToDestTraj = PathPlanner.generatePath(FEEDER.CONSTRAINTS, prepPoint, destPoint);
      var prepToDestCommand = swerve.followTrajectoryCommand(prepToDestTraj, false, true);

      var rotateCommand=new rotateTo(swerve, Rotation2d.fromDegrees(180));

      var raiseArmCommand=new SetArmPose(arm, ArmPose.FEEDER_STATION);

      return new SequentialCommandGroup(
        startToPrepCommand,
        rotateCommand,
        raiseArmCommand,
        prepToDestCommand
      );
}

  private static Command goToDest(Swerve swerve, Arm arm) {
    var startPose = swerve.getPose();
    var endPose = FEEDER.DEST_POSE;
    var traj = MoreMath.createStraightPath(startPose, endPose, FEEDER.CONSTRAINTS);

    var trajCommand = swerve.followTrajectoryCommand(traj, false, true);
    var armCommand = new SetArmPose(arm, ArmPose.FEEDER_STATION);

    return trajCommand.alongWith(armCommand);
  }

  private static boolean inFeederArea(Swerve swerve){
    var currentPose = swerve.getPose();
    var xCheck = MoreMath.within(
      currentPose.getX(), 
      FEEDER.MIN_X, 
      FEEDER.MAX_X);
    var yCheck = MoreMath.within(
      currentPose.getY(), 
      FEEDER.MIN_Y, 
      FEEDER.MAX_Y);
    if(!MoreMath.isBlue()){
      yCheck = MoreMath.within(currentPose.getY(),
      FIELD_CONSTANTS.WIDTH - FEEDER.MIN_Y,
      FIELD_CONSTANTS.WIDTH - FEEDER.MAX_Y);
    }
    return xCheck && yCheck;
  }

  private static boolean isYAligned(Swerve swerve){
    var currentPose = swerve.getPose();
    var destPose = FEEDER.DEST_POSE;

    destPose = MoreMath.transformByAlliance(destPose);

    var yDist = Math.abs(destPose.getY() - currentPose.getY());

    if (yDist < FEEDER.Y_THRESHOLD) {
      return true;
    }
    return false;
  }

  private static boolean isArmRaised(Arm arm){
    var currentPose = arm.getPose();
    var destPose = ArmPose.FEEDER_STATION.val;

    var armDist = Math.abs(currentPose - destPose);

    return armDist < FEEDER.ARM_THRESHOLD;
  }

  private static boolean isRotated(Swerve swerve){
    var currentPose = swerve.getPose();
    var destPose = FEEDER.DEST_POSE;

    destPose = MoreMath.transformByAlliance(destPose);

    var destRad = destPose.getRotation().getRadians();
    var currentRad = currentPose.getRotation().getRadians();

    var closestDestRad = MoreMath.getClosestRad(currentRad, destRad);

    var radDist = Units.radiansToDegrees(Math.abs(closestDestRad - currentRad));

    return radDist < FEEDER.ROT_THRESHOLD;
  }

  private static boolean isClose(Swerve swerve) {
    var currentPose = swerve.getPose();
    var destPose = FEEDER.DEST_POSE;

    destPose = MoreMath.transformByAlliance(destPose);

    return destPose.getTranslation().getDistance(currentPose.getTranslation()) < 1;
  }
}

/* This is what we call "code hoarding" */
//   public static Command rotateThenRaiseWhileMove(Swerve swerve, Arm arm) {
//     Supplier<Command> lambda = () -> {
//       var constraints = new PathConstraints(2, 1);

//       var startPose = swerve.getPose();
//       var midPose = AUTO_ALIGN.FEEDER_PREP_POSE;
//       var endPose = AUTO_ALIGN.FEEDER_DEST_POSE;

//       midPose = MoreMath.transformByAlliance(midPose);
//       endPose = MoreMath.transformByAlliance(endPose);

//       var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
//       var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

//       var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
//       var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation(), 2);

//       var endHeading = midHeading;
//       var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

//       var traj = PathPlanner.generatePath(constraints, startPoint, midPoint, endPoint);
//       // traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

//       return swerve.silentFollowTrajectoryCommand(traj, false, true);
//     };

//     return new rotateTo(swerve, Rotation2d.fromDegrees(180))
//         .andThen(new ProxyCommand(lambda).alongWith(new SetArmPose(arm, ArmPose.FEEDER_STATION)));
//   }

//   public static Command raiseWhileMoveSlowerOption(Swerve swerve, Arm arm) {
//     var startPose = swerve.getPose();
//     var midPose = Constants.AUTO_ALIGN.FEEDER_PREP_POSE;
//     var endPose = Constants.AUTO_ALIGN.FEEDER_DEST_POSE;

//     var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
//     var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

//     var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
//     var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation());

//     var endHeading = midHeading;
//     var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

//     var slowerConstraints = new PathConstraints(0.5, 0.25);

//     var traj1 = PathPlanner.generatePath(AUTO_ALIGN.FEEDER_CONSTRAINTS, startPoint, midPoint);
//     var traj2 = PathPlanner.generatePath(slowerConstraints, midPoint, endPoint);

//     traj1 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj1, DriverStation.getAlliance());
//     traj2 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj2, DriverStation.getAlliance());

//     return swerve.followTrajectoryCommand(traj1, false, false).andThen(
//         new SetArmPose(arm, ArmPose.FEEDER_STATION).alongWith(swerve.followTrajectoryCommand(traj2, false, true)));
//   }

//   public static Command raiseAndMoveOption(Swerve swerve, Arm arm) {
//     var constraints = new PathConstraints(2, 1);
//     var eventMarkers = Collections.singletonList(new EventMarker(Collections.singletonList("raise arm"), 1));

//     var startPose = swerve.getPose();
//     var midPose = AUTO_ALIGN.FEEDER_PREP_POSE;
//     var endPose = AUTO_ALIGN.FEEDER_DEST_POSE;

//     var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
//     var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

//     var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
//     var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation(), 0.5);

//     var endHeading = midHeading;
//     var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

//     var traj = PathPlanner.generatePath(constraints, eventMarkers, startPoint, midPoint, endPoint);
//     traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

//     Map<String, Command> eventMap = Collections.singletonMap("raise arm", new SetArmPose(arm, ArmPose.FEEDER_STATION));

//     return new FollowPathWithEvents(swerve.followTrajectoryCommand(traj, false, true), eventMarkers, eventMap);
//   }

//   public static Command raiseThenMoveOption(Swerve swerve, Arm arm) {
//     var startPose = swerve.getPose();
//     if (Math.abs(startPose.getRotation().getDegrees()) > 150) {
//       return raiseWhileMovingSuboption(swerve, arm);
//     } else {
//       return raiseThenMoveSuboption(swerve, arm);
//     }
//   }

//   public static Command raiseWhileMovingSuboption(Swerve swerve, Arm arm) {
//     var startPose = swerve.getPose();
//     var midPose = Constants.AUTO_ALIGN.FEEDER_PREP_POSE;
//     var endPose = Constants.AUTO_ALIGN.FEEDER_DEST_POSE;

//     var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
//     var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

//     var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
//     var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation());

//     var endHeading = midHeading;
//     var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

//     var traj = PathPlanner.generatePath(AUTO_ALIGN.FEEDER_CONSTRAINTS, startPoint, midPoint, endPoint);

//     traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

//     return swerve.followTrajectoryCommand(traj, false, true).alongWith(new SetArmPose(arm, ArmPose.FEEDER_STATION));
//   }

//   public static Command raiseThenMoveSuboption(Swerve swerve, Arm arm) {
//     var startPose = swerve.getPose();
//     var midPose = Constants.AUTO_ALIGN.FEEDER_PREP_POSE;
//     var endPose = Constants.AUTO_ALIGN.FEEDER_DEST_POSE;

//     var startHeading = midPose.getTranslation().minus(startPose.getTranslation()).getAngle();
//     var startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());

//     var midHeading = endPose.getTranslation().minus(midPose.getTranslation()).getAngle();
//     var midPoint = new PathPoint(midPose.getTranslation(), midHeading, midPose.getRotation());

//     var endHeading = midHeading;
//     var endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());

//     var traj1 = PathPlanner.generatePath(AUTO_ALIGN.FEEDER_CONSTRAINTS, startPoint, midPoint);
//     var traj2 = PathPlanner.generatePath(AUTO_ALIGN.FEEDER_CONSTRAINTS, midPoint, endPoint);

//     traj1 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj1, DriverStation.getAlliance());
//     traj2 = PathPlannerTrajectory.transformTrajectoryForAlliance(traj2, DriverStation.getAlliance());

//     return swerve.followTrajectoryCommand(traj1, false, true).andThen(new SetArmPose(arm, ArmPose.FEEDER_STATION))
//         .andThen(swerve.followTrajectoryCommand(traj2, false, true));
//   }

// }
