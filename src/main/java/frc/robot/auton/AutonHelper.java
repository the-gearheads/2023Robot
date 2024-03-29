package frc.robot.auton;

import java.util.Map;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.wrist.AltWristControl;
import frc.robot.commands.wrist.FloorPickUp;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.CustomProxy;
import frc.robot.util.MoreMath;

public class AutonHelper {

  //format:off
  /* Places a cone on the grid
   * ASSUPTIONS: Cone being held, arm in correct position, alt mode corresponds to setting wrist to 0deg, and default sets it to 90deg
   */
  public static Command getPlaceConeCommand(Subsystems s) {
    return new AltWristControl(s.wrist).raceWith(new SequentialCommandGroup(new WaitCommand(0.25), // Wait for wrist to rotate before dropping cone
        openGrabber(s.grabber))).andThen(new WaitCommand(0.35), // Wait for grabber and gravity to drop cone used to be 0.25
            closeGrabber(s.grabber));
  }

  public static Command getGroundPickUpCommand(Subsystems s) {
    return new SequentialCommandGroup(openGrabber(s.grabber), new WaitCommand(0.25),
        new AltWristControl(s.wrist)
            .raceWith(new SequentialCommandGroup(new WaitCommand(1), closeGrabber(s.grabber), new WaitCommand(1))),
        new WaitCommand(1));
  }

  public static Command stowAnd(Subsystems s, Command... commands) {
    return new ParallelRaceGroup(new SequentialCommandGroup(commands), new SequentialCommandGroup( // Start moving the arm 1 second into the path following
        new WaitCommand(1), new StowArm(s.arm, s.wrist)));
  }

  public static Command openGrabber(Grabber grabber) {
    return new InstantCommand(grabber::open, grabber);
  }

  public static Command closeGrabber(Grabber grabber) {
    return new InstantCommand(grabber::close, grabber);
  }

  public static PathPlannerTrajectory getPathByName(String pathName, PathConstraints constraints) {
    return getPathByName(pathName, constraints, false);
  }

  public static PathPlannerTrajectory getPathByName(String pathName, PathConstraints constraints, boolean reversed) {
    //just curious what it will give us
    Logger.getInstance().recordOutput("Auton/Event Name", DriverStation.getEventName());
    if (DriverStation.getAlliance() == Alliance.Red) {
      var overridePath = PathPlanner.loadPath(Constants.AUTON.EVENT_NAME + "_Red_" + pathName, constraints, reversed);
      if (overridePath != null) {
        Logger.getInstance().recordOutput("Auton/Last Loaded Path", Constants.AUTON.EVENT_NAME + "_Red_" + pathName);
        return overridePath;
      }
    } else {
      var overridePath = PathPlanner.loadPath(Constants.AUTON.EVENT_NAME + "_Blue_" + pathName, constraints, reversed);
      if (overridePath != null) {
        Logger.getInstance().recordOutput("Auton/Last Loaded Path", Constants.AUTON.EVENT_NAME + "_Blue_" + pathName);
        return overridePath;
      }
    }
    /* Both alliances */
    var path = PathPlanner.loadPath(Constants.AUTON.EVENT_NAME + "_" + pathName, constraints, reversed);
    if (path != null) {
      Logger.getInstance().recordOutput("Auton/Last Loaded Path", Constants.AUTON.EVENT_NAME + "_" + pathName);
      return path;
    }

    /* Actual path */
    path = PathPlanner.loadPath(pathName, constraints, reversed);
    if (path == null) {
      DriverStation.reportError("Failed to load path: " + pathName, true);
    }

    Logger.getInstance().recordOutput("Auton/Last Loaded Path", pathName);
    return path;
  }

  public static Command getCommandForPath(String pathName, boolean resetOdometry, PathConstraints constraints,
      Swerve swerve) {
    return getCommandForPath(pathName, resetOdometry, constraints, swerve, false);
  }

  public static Command getCommandForPath(String pathName, boolean resetOdometry, PathConstraints constraints,
      Swerve swerve, boolean reversed) {
    return new CustomProxy(() -> {
      //   var path = PathPlanner.loadPath(pathName, constraints);
      var path = getPathByName(pathName, constraints, reversed);
      return swerve.followTrajectoryCommand(path, resetOdometry, true);
    });
  }

  public static Command setInitPose(Subsystems s, String pathName) {
    return new InstantCommand(() -> {
      PathPlannerTrajectory path = PathPlanner.loadPath(pathName, Constants.AUTON.SLOW_CONSTRAINTS);
      path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
      var initPose = path.getInitialPose();
      s.swerve.setPose(initPose);
    });
  }

  public static Command setInitRot(Swerve swerve, String pathName) {
    return new InstantCommand(() -> {
      PathPlannerTrajectory path = PathPlanner.loadPath(pathName, Constants.AUTON.SLOW_CONSTRAINTS);
      path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
      var initRot = path.getInitialPose().getRotation();
      swerve.setPose(new Pose2d(swerve.getPose().getX(), swerve.getPose().getY(), initRot));
    });
  }

  public static CustomProxy goToPathDesination(Swerve swerve, String pathName, PathConstraints constraints) {
    return new CustomProxy(() -> {
      PathPlannerTrajectory path = PathPlanner.loadPath(pathName, constraints);
      path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
      var finalState = path.getEndState();
      var finalPose =
          new Pose2d(finalState.poseMeters.getX(), finalState.poseMeters.getY(), finalState.holonomicRotation);

      return swerve.goTo(finalPose, constraints);
    }, swerve);
  }

  public static CustomProxy goToGridAlignment(Swerve swerve, Translation2d blueDest, Translation2d redDest,
      Rotation2d rot, PathConstraints constraints) {
    return new CustomProxy(() -> {
      if (MoreMath.isBlue()) {
        return swerve.goTo(new Pose2d(blueDest, rot), constraints);
      } else {
        return swerve.goTo(new Pose2d(redDest, rot), constraints);
      }
    }, swerve);
  }

  public static Command followWithEvents(String pathName, Map<String, Command> eventMap, boolean resetOdometry,
      PathConstraints constrainsts, Swerve swerve, boolean reversed) {
    var path = AutonHelper.getPathByName(pathName, constrainsts);
    var pathCommand = AutonHelper.getCommandForPath(pathName, resetOdometry, constrainsts, swerve);

    return new FollowPathWithEvents(pathCommand, path.getMarkers(), eventMap);
  }

  public static Command pickupCone(Subsystems s) {
    return (AutonHelper.closeGrabber(s.grabber).andThen(new WaitCommand(0.25)))
        .raceWith(new FloorPickUp(s.arm, s.wrist));
  }
}
//format:on
