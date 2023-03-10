package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServerThread;
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
import frc.robot.auton.AutonHelper;

// format: off
public class AutonPaths {
  /* Places cone in high node, goes to GP1 to pick up a cube, places it in low hybrid node
   * ASSUMPTIONS: Cone preloaded, arm already inside robot, in inert pos, pos constants correct
   */

  private static PathConstraints defaultConstraints = Constants.AUTON.MID_CONSTRAINTS;

  public static Command InertN4PlaceThenDock(Subsystems s) {
    return new SequentialCommandGroup(
        AutonHelper.setInitPose(s, "InertN4-StartN4"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, 
          AutonHelper.getCommandForPath("StartN4-PrepareDock", false, Constants.AUTON.DOCK_CONSTRAINTS, s.swerve),
          new AutoBalance(s.swerve)));
  }

  public static Command InertN1PlaceThenExplore(Subsystems s) {
    return new SequentialCommandGroup(
        AutonHelper.setInitPose(s, "InertN1-StartN1"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN1-Explore", false, defaultConstraints, s.swerve))
    );
  }

  public static Command InertN9PlaceThenExplore(Subsystems s) {
    return new SequentialCommandGroup(
        AutonHelper.setInitPose(s, "InertN9-StartN9"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN9-Explore", false, defaultConstraints, s.swerve))
    );
  }

  /* NOT USED FOR WAYNE STATE (add back later)----------------------------------------------------------------------- */
    // public static CommandBase InertN4ExploreOverStationDock(Subsystems s) {
  //   return new SequentialCommandGroup(setInitPose(s, "InertN4-StartN4"), new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

  //       getPlaceConeCommand(s),

  //       stowAnd(s,
  //           getCommandForPath("StartN4-ExploreOverStation", false, defaultConstraints, s.swerve),
  //           getCommandForPath("ExploreOverStation-Dock", false, defaultConstraints, s.swerve),
  //           new AutoBalance(s.swerve)));
  // }

  // public static CommandBase InertN1PlaceThenDock(Subsystems s) {
  //   return new SequentialCommandGroup(
  //       // Move forward
  //       // new ParallelCommandGroup(
  //       new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),
  //       // ),

  //       // place game piece
  //       getPlaceConeCommand(s),

  //       // Go to charging station
  //       stowAnd(s, getCommandForPath("StartN1-Explore-PrepareDock", false, defaultConstraints, s.swerve),
  //           new AutoBalance(s.swerve)));
  // }

  // public static CommandBase InertN1TwoConePath(Subsystems s) {
  //   return new SequentialCommandGroup(
  //     setInitPose(s, "InertN1-StartN1"),
  //       // Move forward
  //       new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

  //       // place game piece
  //       getPlaceConeCommand(s),

  //       // Go get game piece
  //       new ParallelCommandGroup(new SetArmPose(s.arm, ArmPose.FLOOR),
  //         getCommandForPath("StartN1-GamePiece1", false, defaultConstraints, s.swerve)
  //       ),

  //       // pick up game piece
  //       getGroundPickUpCommand(s),
        

  //       // go back to grid node 3 inert
  //       new ParallelCommandGroup(
  //           getCommandForPath("GamePiece1-StartN3", false, defaultConstraints, s.swerve),
  //           new WaitCommand(1).andThen(new SetArmPose(s.arm, ArmPose.HIGH_NODE))),

  //       // Place game piece
  //       getPlaceConeCommand(s));
  // }

  // public static CommandBase InertN9TwoConePath(Subsystems s) {
  //   return new SequentialCommandGroup(
  //     setInitPose(s, "InertN9-StartN9"),
  //       // Move forward
  //       new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

  //       // place game piece
  //       getPlaceConeCommand(s),

  //       // Go get game piece
  //       stowAnd(s, getCommandForPath("StartN9-GamePiece9", false, defaultConstraints, s.swerve)),

  //       // pick up game piece
  //       getGroundPickUpCommand(s),

  //       // go back to grid node 3 inert
  //       new ParallelCommandGroup(
  //           getCommandForPath("GamePiece9-StartN7", false, defaultConstraints, s.swerve),
  //           new SetArmPose(s.arm, ArmPose.HIGH_NODE)),

  //       // Place game piece
  //       getPlaceConeCommand(s));
  // }

  // public static CommandBase InertN9PlaceThenDock(Subsystems s) {
  //   return new SequentialCommandGroup(
  //       // Move forward
  //       // new ParallelCommandGroup(
  //       new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),
  //       // ),

  //       // place game piece
  //       getPlaceConeCommand(s),

  //       // Go to charging station
  //       stowAnd(s, getCommandForPath("StartN9-Explore-PrepareDock", false, defaultConstraints, s.swerve)),

  //       new AutoBalance(s.swerve)

  //   );
  // }

  // public static CommandBase InertN1GrabThenDock(Subsystems s) {
  //   return new SequentialCommandGroup(
  //       // Move forward
  //       new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

  //       // place game piece
  //       getPlaceConeCommand(s),

  //       // Go get game piece
  //       stowAnd(s, getCommandForPath("StartN1-GamePiece1", false, defaultConstraints, s.swerve)),

  //       // pick up game piece
  //       getGroundPickUpCommand(s),

  //       // go back to grid node 3 inert
  //       new ParallelCommandGroup(
  //           getCommandForPath("GamePiece1-PrepareDock", false, defaultConstraints, s.swerve),
  //           new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT)),

  //       // run autobalance 
  //       new AutoBalance(s.swerve));

  // }

  // public static CommandBase InertN9GrabThenDock(Subsystems s) {
  //   return new SequentialCommandGroup(
  //       // Move forward
  //       new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

  //       // place game piece
  //       getPlaceConeCommand(s),

  //       // Go get game piece
  //       stowAnd(s, getCommandForPath("StartN9-GamePiece9", false, defaultConstraints, s.swerve)),

  //       // pick up game piece
  //       getGroundPickUpCommand(s),

  //       // go back to grid node 3 inert
  //       new ParallelCommandGroup(
  //           getCommandForPath("GamePiece9-PrepareDock", false, defaultConstraints, s.swerve),
  //           new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT)),

  //       new AutoBalance(s.swerve));

  // }

  // public static CommandBase InertN9StraightToDock(Subsystems s) {
  //   return new SequentialCommandGroup(
  //       // Move forward
  //       new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

  //       // place game piece
  //       getPlaceConeCommand(s),

  //       //go to autobalance pose
  //       stowAnd(s, getCommandForPath("InertN9-PrepareDock", false, defaultConstraints, s.swerve)),

  //       // run autobalance here
  //       new AutoBalance(s.swerve));
  // }

  // public static CommandBase InertN1StraightToDock(Subsystems s) {
  //   return new SequentialCommandGroup(
  //       // Move forward
  //       new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

  //       // place game piece
  //       getPlaceConeCommand(s),

  //       //go to autobalance pose
  //       stowAnd(s, getCommandForPath("InertN1-PrepareDock", false, defaultConstraints, s.swerve)),

  //       // run autobalance here
  //       new AutoBalance(s.swerve));
  // }
}

// format: on
