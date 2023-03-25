package frc.robot.auton;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.AutoBalDriveToPivot;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.rotateTo;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.CustomProxy;

// format: off
public class AutonPaths {
  /* Places cone in high node, goes to GP1 to pick up a cube, places it in low hybrid node
   * ASSUMPTIONS: Cone preloaded, arm already inside robot, in inert pos, pos constants correct
   */

  private static PathConstraints defaultConstraints = Constants.AUTON.MID_CONSTRAINTS;

  public static Command InertN4PlaceThenDock(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN4-StartN4"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s,
            AutonHelper.getCommandForPath("StartN4-PrepareDock", false, Constants.AUTON.DOCK_CONSTRAINTS, s.swerve),
            new AutoBalance(s.swerve)));
  }

  public static Command InertN1Place(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN4-StartN4"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s));
  }

  public static Command InertN1PlaceThenExplore(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN1-StartN1"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN1-Explore", false, defaultConstraints, s.swerve)));
  }

  public static Command InertN9PlaceThenExplore(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN9-StartN9"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN9-Explore", false, defaultConstraints, s.swerve)));
  }

  public static Command InertN1SafeTwoCone(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN1-StartN1"),
        AutonHelper.setInitPose(s, "InertN1-StartN1"),
        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        new ParallelCommandGroup(
            AutonHelper.getCommandForPath("StartN1-GamePiece1-No-Turn", false, Constants.AUTON.DOUBLE_CONE, s.swerve),
            new SequentialCommandGroup( // Start moving the arm 1 second into the path following
                new WaitCommand(1), new SetArmPose(s.arm, ArmPose.FLOOR))),
        new rotateTo(s.swerve, Rotation2d.fromDegrees(0)),

        AutonHelper.getGroundPickUpCommand(s),

        new rotateTo(s.swerve, Rotation2d.fromDegrees(180)),

        AutonHelper.stowAnd(s,
            AutonHelper.getCommandForPath("GamePiece1-InertN3-No-Turn", false, defaultConstraints, s.swerve))

    // places cone
    // new SetArmPose(s.arm, ArmPose.HIGH_NODE),

    // AutonHelper.getCommandForPath("InertN3-StartN3", false, defaultConstraints, s.swerve),

    // AutonHelper.getPlaceConeCommand(s)
    );
  }

  /* NOT USED FOR WAYNE STATE (add back later)----------------------------------------------------------------------- */

  public static Command InertN4GrabThenDock(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN4-StartN4"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        new CustomProxy(() -> {
          var path = AutonHelper.getPathByName("StartN4-ExploreOverStation-GamePiece2", defaultConstraints);
          var pathCommand = AutonHelper.getCommandForPath("StartN4-ExploreOverStation-GamePiece2", false,
              defaultConstraints, s.swerve);
          HashMap<String, Command> eventMap = new HashMap<>();
          eventMap.put("stow_arm", new SetArmPose(s.arm, ArmPose.FLOOR));

          return new FollowPathWithEvents(pathCommand, path.getMarkers(), eventMap);
        }),

        AutonHelper.getGroundPickUpCommand(s), new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),

        AutonHelper.stowAnd(s,
            AutonHelper.getCommandForPath("ExploreOverStation-Dock", false, defaultConstraints, s.swerve)),

        new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve)));
  }

  public static Command InertN1TwoConePrime(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN1-StartN1"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        new ParallelCommandGroup(
            AutonHelper.getCommandForPath("StartN1-GamePiece1-Prime", false, defaultConstraints, s.swerve),
            new SequentialCommandGroup( // Start moving the arm 1 second into the path following
                new WaitCommand(1), new SetArmPose(s.arm, ArmPose.FLOOR))),

        AutonHelper.getGroundPickUpCommand(s), new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),

        AutonHelper.stowAnd(s,
            AutonHelper.getCommandForPath("GamePiece1-Inert3-Prime", false, defaultConstraints, s.swerve)));
  }

  public static Command InertN9TwoCone(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN9-StartN9"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        new CustomProxy(() -> {
          var path = AutonHelper.getPathByName("StartN9-GamePiece1", defaultConstraints);
          var pathCommand = AutonHelper.getCommandForPath("StartN9-GamePiece1", false, defaultConstraints, s.swerve);
          HashMap<String, Command> eventMap = new HashMap<>();
          eventMap.put("arm_toggle", new SetArmPose(s.arm, ArmPose.FLOOR));

          return new FollowPathWithEvents(pathCommand, path.getMarkers(), eventMap);
        }),

        new rotateTo(s.swerve, Rotation2d.fromDegrees(0)), AutonHelper.getGroundPickUpCommand(s),
        new rotateTo(s.swerve, Rotation2d.fromDegrees(180)),

        new CustomProxy(() -> {
          var path = AutonHelper.getPathByName("GamePiece1-StartN9", defaultConstraints);
          var pathCommand = AutonHelper.getCommandForPath("GamePiece1-StartN9", false, defaultConstraints, s.swerve);
          HashMap<String, Command> eventMap = new HashMap<>();
          eventMap.put("arm_toggle", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));

          return new FollowPathWithEvents(pathCommand, path.getMarkers(), eventMap);
        }));

    // place game piece
    // AutonHelper.getPlaceConeCommand(s));
  }
  // public static CommandBase InertN4ExploreOverStationDock(Subsystems s) {
  //   return new SequentialCommandGroup(setInitPose(s, "InertN4-StartN4"), new SetArmPose(s.arm, ArmPose.HIGH_NODE),
  //       getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

  //       getPlaceConeCommand(s),

  //       stowAnd(s,
  //           getCommandForPath("StartN4-ExploreOverStation", false, defaultConstraints, s.swerve),
  //           getCommandForPath("ExploreOverStation-Dock", false, defaultConstraints, s.swerve),
  //           new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve))
  // );
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
  //           new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve)));
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

  //       new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve)

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
  //       new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve));

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

  //       new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve));

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
  //       new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve));
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
  //       new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve));
  // }
}

// format: on
