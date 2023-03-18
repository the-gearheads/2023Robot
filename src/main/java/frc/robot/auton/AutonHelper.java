package frc.robot.auton;

import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.wrist.AltWristControl;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.CustomProxy;

public class AutonHelper {

  //format:off
  /* Places a cone on the grid
   * ASSUPTIONS: Cone being held, arm in correct position, alt mode corresponds to setting wrist to 0deg, and default sets it to 90deg
   */
  public static Command getPlaceConeCommand(Subsystems s) {
    return new AltWristControl(s.wrist).raceWith(new SequentialCommandGroup(new WaitCommand(0.5), // Wait for wrist to rotate before dropping cone
        getGrabberOpenCommand(s.grabber))).andThen(new WaitCommand(0.5), // Wait for grabber and gravity to drop cone
            getGrabberCloseCommand(s.grabber));
  }

  public static Command getGroundPickUpCommand(Subsystems s) {
    return  new SequentialCommandGroup(getGrabberOpenCommand(s.grabber), new WaitCommand(0.25),
        new AltWristControl(s.wrist).raceWith(
            new SequentialCommandGroup(
              new WaitCommand(1), 
              getGrabberCloseCommand(s.grabber), 
              new WaitCommand(1))),
        new WaitCommand(1));
  }

  public static Command stowAnd(Subsystems s, Command... commands) {
    return new ParallelRaceGroup(new SequentialCommandGroup(commands), new SequentialCommandGroup( // Start moving the arm 1 second into the path following
        new WaitCommand(1), new StowArm(s.arm, s.wrist)));
  }

  public static Command getGrabberOpenCommand(Grabber grabber) {
    return new InstantCommand(() -> {
      grabber.open();
    }, grabber);
  }

  public static Command getGrabberCloseCommand(Grabber grabber) {
    return new InstantCommand(() -> {
      grabber.close();
    }, grabber);
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
}
//format:on
