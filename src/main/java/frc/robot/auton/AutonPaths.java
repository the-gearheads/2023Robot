package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.wrist.AltWristControl;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.drive.Swerve;

// format: off
public class AutonPaths {
  /* Places cone in high node, goes to GP1 to pick up a cube, places it in low hybrid node
   * ASSUMPTIONS: Cone preloaded, arm already inside robot, in inert pos, pos constants correct
   */

  private static PathConstraints defaultConstraints = Constants.AUTON.MID_CONSTRAINTS;

  public static Command InertN4PlaceThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        setInitPose(s, "InertN4-StartN4"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        // place game piece
        getPlaceConeCommand(s),

        stowAnd(s, getCommandForPath("StartN4-PrepareDock", false, Constants.AUTON.DOCK_CONSTRAINTS, s.swerve),
            new AutoBalance(s.swerve)));
  }

  public static CommandBase InertN4ExploreOverStationDock(Subsystems s) {
    return new SequentialCommandGroup(setInitPose(s, "InertN4-StartN4"), new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        getPlaceConeCommand(s),

        stowAnd(s,
            getCommandForPath("StartN4-ExploreOverStation", false, defaultConstraints, s.swerve),
            getCommandForPath("ExploreOverStation-Dock", false, defaultConstraints, s.swerve),
            new AutoBalance(s.swerve)));
  }

  public static CommandBase InertN1PlaceThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        // new ParallelCommandGroup(
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),
        // ),

        // place game piece
        getPlaceConeCommand(s),

        // Go to charging station
        stowAnd(s, getCommandForPath("StartN1-Explore-PrepareDock", false, defaultConstraints, s.swerve),
            new AutoBalance(s.swerve)));
  }

  public static CommandBase InertN1TwoConePath(Subsystems s) {
    return new SequentialCommandGroup(
      setInitPose(s, "InertN1-StartN1"),
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        getPlaceConeCommand(s),

        // Go get game piece
        stowAnd(s, getCommandForPath("StartN1-GamePiece1", false, defaultConstraints, s.swerve)),

        // pick up game piece
        getGroundPickUpCommand(s),

        // go back to grid node 3 inert
        new ParallelCommandGroup(
            getCommandForPath("GamePiece1-StartN3", false, defaultConstraints, s.swerve),
            new SetArmPose(s.arm, ArmPose.HIGH_NODE)),

        // Place game piece
        getPlaceConeCommand(s));
  }

  public static CommandBase InertN9TwoConePath(Subsystems s) {
    return new SequentialCommandGroup(
      setInitPose(s, "InertN9-StartN9"),
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

        // place game piece
        getPlaceConeCommand(s),

        // Go get game piece
        stowAnd(s, getCommandForPath("StartN9-GamePiece9", false, defaultConstraints, s.swerve)),

        // pick up game piece
        getGroundPickUpCommand(s),

        // go back to grid node 3 inert
        new ParallelCommandGroup(
            getCommandForPath("GamePiece9-StartN7", false, defaultConstraints, s.swerve),
            new SetArmPose(s.arm, ArmPose.HIGH_NODE)),

        // Place game piece
        getPlaceConeCommand(s));
  }

  public static CommandBase InertN9PlaceThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        // new ParallelCommandGroup(
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),
        // ),

        // place game piece
        getPlaceConeCommand(s),

        // Go to charging station
        stowAnd(s, getCommandForPath("StartN9-Explore-PrepareDock", false, defaultConstraints, s.swerve)),

        new AutoBalance(s.swerve)

    );
  }

  public static CommandBase InertN1GrabThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        getPlaceConeCommand(s),

        // Go get game piece
        stowAnd(s, getCommandForPath("StartN1-GamePiece1", false, defaultConstraints, s.swerve)),

        // pick up game piece
        getGroundPickUpCommand(s),

        // go back to grid node 3 inert
        new ParallelCommandGroup(
            getCommandForPath("GamePiece1-PrepareDock", false, defaultConstraints, s.swerve),
            new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT)),

        // run autobalance 
        new AutoBalance(s.swerve));

  }

  public static CommandBase InertN9GrabThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

        // place game piece
        getPlaceConeCommand(s),

        // Go get game piece
        stowAnd(s, getCommandForPath("StartN9-GamePiece9", false, defaultConstraints, s.swerve)),

        // pick up game piece
        getGroundPickUpCommand(s),

        // go back to grid node 3 inert
        new ParallelCommandGroup(
            getCommandForPath("GamePiece9-PrepareDock", false, defaultConstraints, s.swerve),
            new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT)),

        new AutoBalance(s.swerve));

  }

  public static CommandBase InertN9StraightToDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

        // place game piece
        getPlaceConeCommand(s),

        //go to autobalance pose
        stowAnd(s, getCommandForPath("InertN9-PrepareDock", false, defaultConstraints, s.swerve)),

        // run autobalance here
        new AutoBalance(s.swerve));
  }

  public static CommandBase InertN1StraightToDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        getPlaceConeCommand(s),

        //go to autobalance pose
        stowAnd(s, getCommandForPath("InertN1-PrepareDock", false, defaultConstraints, s.swerve)),

        // run autobalance here
        new AutoBalance(s.swerve));
  }

  /* Places a cone on the grid
   * ASSUPTIONS: Cone being held, arm in correct position, alt mode corresponds to setting wrist to 0deg, and default sets it to 90deg
   */
  public static Command getPlaceConeCommand(Subsystems s) {
    return new AltWristControl(s.wrist).raceWith(new SequentialCommandGroup(
        new WaitCommand(0.25), // Wait for wrist to rotate before dropping cone
        getGrabberOpenCommand(s.grabber),
        new WaitCommand(0.25), // Wait for grabber and gravity to drop cone
        getGrabberCloseCommand(s.grabber)
    ));
  }

  public static Command getGroundPickUpCommand(Subsystems s) {
    return new SequentialCommandGroup(getGrabberOpenCommand(s.grabber), new WaitCommand(0.25),
        new AltWristControl(s.wrist).raceWith(new SequentialCommandGroup(new WaitCommand(0.25),
            getGrabberCloseCommand(s.grabber), new WaitCommand(0.25))));
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

      if (DriverStation.getAlliance() == Alliance.Red) {
        var overridePath = PathPlanner.loadPath(DriverStation.getEventName() + "Red_" + pathName, constraints);
        if (overridePath != null)
          return overridePath;
      } else {
        var overridePath = PathPlanner.loadPath(DriverStation.getEventName() + "Blue_" + pathName, constraints);
        if (overridePath != null)
          return overridePath;
      }
      /* Both alliances */
      var path = PathPlanner.loadPath(DriverStation.getEventName() + "_" + pathName, constraints);
      if (path != null)
        return path;


    /* Actual path */
    path = PathPlanner.loadPath(pathName, constraints);
    if (path == null) {
      DriverStation.reportError("Failed to load path: " + pathName, true);
    }
    return path;
  }

  public static Command getCommandForPath(String pathName, boolean resetOdometry, PathConstraints constraints,
      Swerve swerve) {
    return new ProxyCommand(()->{
      var path = PathPlanner.loadPath(pathName, constraints);
      // var path = getPathByName(pathName, constraints);
      return swerve.silentFollowTrajectoryCommand(path, resetOdometry, true);
    }); 
  }

  public static Command setInitPose(Subsystems s, String pathName) {
    return new InstantCommand(() -> {
      PathPlannerTrajectory path = PathPlanner.loadPath(pathName, defaultConstraints);
      var initPose = path.getInitialPose();
      s.swerve.setPose(initPose);
    });
  }

}

// format: on
