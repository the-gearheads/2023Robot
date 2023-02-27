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
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState.WristStateType;

public class AutonPaths {
  /* Places cone in high node, goes to GP1 to pick up a cube, places it in low hybrid node
   * ASSUMPTIONS: Cone preloaded, arm already inside robot, in inert pos, pos constants correct
   */
  public static CommandBase getTestPlacePath(Subsystems s) {
    return new SequentialCommandGroup(
      // Place cone
      new SetArmPose(s.arm, ArmPose.HIGH_NODE),
      getCommandForPath("Inert_To_Start", true, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
      getPlaceConeCommand(s),

      // Get a cube
      new ParallelCommandGroup(
        getCommandForPath("Start_To_Game_Piece_1", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
        new SequentialCommandGroup( // Start moving the arm 1 second into the path following
          new WaitCommand(1),
          new SetArmPose(s.arm, ArmPose.FLOOR)
        )
      ),
      getGrabberCloseCommand(s.grabber),
      
      // Prepare to place it
      new ParallelCommandGroup(
        getCommandForPath("Game_Piece_1_To_Start", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
        new SetArmPose(s.arm, ArmPose.LOW_NODE)
      ),

      // Place it
      getGrabberOpenCommand(s.grabber),
      new WaitCommand(0.5),
      new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT)
    );
  }

  /* Places a cone on the grid
   * ASSUPTIONS: Cone being held, arm in correct position, alt mode corresponds to setting wrist to 0deg, and default sets it to 90deg
   */
  public static Command getPlaceConeCommand(Subsystems s) {
    return new SequentialCommandGroup(
      setWristModeCommand(s.wrist, WristStateType.ALT),
      new WaitCommand(0.5), // Wait for wrist to rotate before dropping cone
      getGrabberOpenCommand(s.grabber),
      new WaitCommand(0.5), // Wait for grabber and gravity to drop cone
      getGrabberCloseCommand(s.grabber),
      setWristModeCommand(s.wrist, WristStateType.DEFAULT), // Move wrist back
      new WaitCommand(0.5) // Possibly not needed
    );
  }

  public static Command getGrabberOpenCommand(Grabber grabber) {
    return new InstantCommand(()->{
      grabber.open();
    }, grabber);
  }

  public static Command getGrabberCloseCommand(Grabber grabber) {
    return new InstantCommand(()->{
      grabber.open();
    }, grabber);
  }

  public static Command setWristModeCommand(Wrist wrist, WristStateType type) {
    return new InstantCommand(()->{
      wrist.setGoalByType(type);
    });
  }

  public static Command getCommandForPath(String pathName, boolean resetOdometry, PathConstraints constraints, Swerve swerve) {
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

}