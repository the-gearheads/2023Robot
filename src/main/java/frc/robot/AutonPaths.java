package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.wrist.AltWristControl;
import frc.robot.commands.wrist.DefaultWristControl;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.drive.Swerve;

// format: off
public class AutonPaths {
  /* Places cone in high node, goes to GP1 to pick up a cube, places it in low hybrid node
   * ASSUMPTIONS: Cone preloaded, arm already inside robot, in inert pos, pos constants correct
   */

  public static Command InertN4PlaceThenDock(Subsystems s){
    return new SequentialCommandGroup(
        setInitPose(s,"InertN4-StartN4"),
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN4-StartN4", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),

        // place game piece
        getPlaceConeCommand(s),
        
        new ParallelCommandGroup(
          getCommandForPath("StartN4-PrepareDock", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
          new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT)));
  }
  public static CommandBase InertN1TwoConePath(Subsystems s) {
    return new SequentialCommandGroup(
        setInitPose(s,"InertN1-StartN1"),
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN1-StartN1", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),

        // place game piece
        getPlaceConeCommand(s),

        // Go get game piece
        new ParallelCommandGroup(
            getCommandForPath("StartN1-GamePiece1", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SequentialCommandGroup( // Start moving the arm 1 second into the path following
                new WaitCommand(1), new SetArmPose(s.arm, ArmPose.FLOOR))),

        // pick up game piece
        getGroundPickUpCommand(s),

        // go back to grid node 3 inert
        new ParallelCommandGroup(
            getCommandForPath("GamePiece1-StartN3", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SetArmPose(s.arm, ArmPose.HIGH_NODE)),

        // Place game piece
        getPlaceConeCommand(s));
  }

  public static CommandBase InertN9TwoConePath(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        new ParallelCommandGroup(new SetArmPose(s.arm, ArmPose.HIGH_NODE),
            getCommandForPath("InertN9-StartN9", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve)),

        // place game piece
        getPlaceConeCommand(s),

        // Go get game piece
        new ParallelCommandGroup(
            getCommandForPath("StartN9-GamePiece9", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SequentialCommandGroup( // Start moving the arm 1 second into the path following
                new WaitCommand(1), new SetArmPose(s.arm, ArmPose.FLOOR))),

        // pick up game piece
        getGroundPickUpCommand(s),

        // go back to grid node 3 inert
        new ParallelCommandGroup(
            getCommandForPath("GamePiece9-StartN7", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SetArmPose(s.arm, ArmPose.HIGH_NODE)),

        // Place game piece
        getPlaceConeCommand(s));
  }

  public static CommandBase InertN1PlaceThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        // new ParallelCommandGroup(
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN1-StartN1", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
        // ),

        // place game piece
        getPlaceConeCommand(s),

        // Go to charging station
        new ParallelCommandGroup(
            getCommandForPath("StartN1-Explore-PrepareDock", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SequentialCommandGroup( // Start moving the arm 1 second into the path following
                new WaitCommand(1), new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT))));

        // Add autobalance here.
  }

  public static CommandBase InertN9PlaceThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        // new ParallelCommandGroup(
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        getCommandForPath("InertN9-StartN9", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
        // ),

        // place game piece
        getPlaceConeCommand(s),

        // Go to charging station
        new ParallelCommandGroup(
            getCommandForPath("StartN9-Explore-PrepareDock", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SequentialCommandGroup( // Start moving the arm 1 second into the path following
                new WaitCommand(1), new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT))));
  }

  public static CommandBase InertN1GrabThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        new ParallelCommandGroup(new SetArmPose(s.arm, ArmPose.HIGH_NODE),
            getCommandForPath("InertN1-StartN1", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve)),

        // place game piece
        getPlaceConeCommand(s),

        // Go get game piece
        new ParallelCommandGroup(
            getCommandForPath("StartN1-GamePiece1", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SequentialCommandGroup( // Start moving the arm 1 second into the path following
                new WaitCommand(1), new SetArmPose(s.arm, ArmPose.FLOOR))),

        // pick up game piece
        getGroundPickUpCommand(s),

        // go back to grid node 3 inert
        new ParallelCommandGroup(
            getCommandForPath("GamePiece1-PrepareDock", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT))

        // run autobalance      
      );

  }

  public static CommandBase InertN9GrabThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        // Move forward
        new ParallelCommandGroup(new SetArmPose(s.arm, ArmPose.HIGH_NODE),
            getCommandForPath("InertN9-StartN9", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve)),

        // place game piece
        getPlaceConeCommand(s),

        // Go get game piece
        new ParallelCommandGroup(
            getCommandForPath("StartN9-GamePiece9", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SequentialCommandGroup( // Start moving the arm 1 second into the path following
                new WaitCommand(1), new SetArmPose(s.arm, ArmPose.FLOOR))),

        // pick up game piece
        getGroundPickUpCommand(s),

        // go back to grid node 3 inert
        new ParallelCommandGroup(
            getCommandForPath("GamePiece9-PrepareDock", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
            new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT)));

        // run autobalance here
  }

  public static CommandBase Inert9StraightToDock(Subsystems s) {
    return new SequentialCommandGroup(
      // Move forward
      new ParallelCommandGroup(new SetArmPose(s.arm, ArmPose.HIGH_NODE),
          getCommandForPath("InertN9-StartN9", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve)),

      // place game piece
      getPlaceConeCommand(s),

      //go to autobalance pose
      new ParallelCommandGroup(new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),
        getCommandForPath("InertN9-PrepareDock", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve)  
      )

      // run autobalance here
    );
  }

  public static CommandBase Inert1StraightToDock(Subsystems s) {
    return new SequentialCommandGroup(
      // Move forward
      new ParallelCommandGroup(new SetArmPose(s.arm, ArmPose.HIGH_NODE),
          getCommandForPath("InertN1-StartN1", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve)),

      // place game piece
      getPlaceConeCommand(s),

      //go to autobalance pose
      new ParallelCommandGroup(new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),
        getCommandForPath("InertN1-PrepareDock", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve)  
      )

      // run autobalance here
    );
  }

  /* Places a cone on the grid
   * ASSUPTIONS: Cone being held, arm in correct position, alt mode corresponds to setting wrist to 0deg, and default sets it to 90deg
   */
  public static Command getPlaceConeCommand(Subsystems s) {
    return new SequentialCommandGroup(new AltWristControl(s.wrist), new WaitCommand(0.5), // Wait for wrist to rotate before dropping cone
        getGrabberOpenCommand(s.grabber), new WaitCommand(0.5), // Wait for grabber and gravity to drop cone
        getGrabberCloseCommand(s.grabber), new DefaultWristControl(s.wrist));
  }

  public static Command getGroundPickUpCommand(Subsystems s) {
    return new SequentialCommandGroup(getGrabberOpenCommand(s.grabber), new WaitCommand(0.5),
        new AltWristControl(s.wrist), new WaitCommand(0.5), getGrabberCloseCommand(s.grabber), new WaitCommand(0.5),
        new DefaultWristControl(s.wrist));
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

  public static Command getCommandForPath(String pathName, boolean resetOdometry, PathConstraints constraints,
      Swerve swerve) {
    PathPlannerTrajectory path = PathPlanner.loadPath(pathName, constraints);
    if (path == null) {
      DriverStation.reportError("Failed to load path: " + pathName, true);
      return new InstantCommand(() -> {
        DriverStation.reportError("Tried to execute path that failed to load! Path name: " + pathName, true);
      });
    }
    Command forwardCommand = swerve.followTrajectoryCommand(path, resetOdometry, true);
    return forwardCommand;
  }
  
  public static Command setInitPose(Subsystems s, String pathName){
    return new InstantCommand(()->{
      PathPlannerTrajectory path = PathPlanner.loadPath(pathName, Constants.AUTON.SLOW_CONSTRAINTS);
      var initPose = path.getInitialPose();
      s.swerve.setPose(initPose);
    });
  }

}

// format: on
