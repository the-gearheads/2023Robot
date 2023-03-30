package frc.robot.auton;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.AutoBalDriveToPivot;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.rotateTo;
import frc.robot.commands.drive.autoalign.Grid;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.commands.wrist.FloorPickUp;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;
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
            new AutoBalance(s.swerve))
        )
        .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE));
  }

  public static Command InertN1Place(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN4-StartN4"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s))
        .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE));
  }

  public static Command InertN1PlaceThenExplore(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN1-StartN1"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN1-Explore", false, defaultConstraints, s.swerve)))
        .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE));
  }

  public static Command InertN9PlaceThenExplore(Subsystems s) {
    return new SequentialCommandGroup(AutonHelper.setInitPose(s, "InertN9-StartN9"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN9-Explore", false, defaultConstraints, s.swerve)))
        .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE));
  }

    public static Command TwoGamePieceNoBump(Subsystems s){
      return new SequentialCommandGroup(
        new InstantCommand(()->{
          SmartDashboard.putNumber("auton phase", 1);
        }),
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        new InstantCommand(()->{
          SmartDashboard.putNumber("auton phase", 2);
        }),
        new InstantCommand(()->{
          s.swerve.setPose(new Pose2d(s.swerve.getPose().getX(), s.swerve.getPose().getY(), Rotation2d.fromDegrees(180)));
        }, s.swerve),

        new CustomProxy(()->{
          var destTrans = Grid.LEFT_GRID.leftCol.high;
          return s.swerve.goTo(new Pose2d(destTrans, Rotation2d.fromDegrees(180)), Constants.AUTON.SLOW_CONSTRAINTS);
        }, s.swerve),

        AutonHelper.getPlaceConeCommand(s),

        new CustomProxy(()->{
          return twoGamePieceNoBumpPathPickUpCubeProxy(s);
        }),

        (AutonHelper.closeGrabber(s.grabber).andThen(new WaitCommand(0.25)))
        .raceWith(new FloorPickUp(s.arm, s.wrist))
        ,

        new CustomProxy(()->{
          return twoGamePieceNoBumpPathPlaceCubeProxy(s);
        })

      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
    }

  public static Command twoGamePieceNoBumpPathPickUpCubeProxy(Subsystems s){
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    eventMap.put("pickup", AutonHelper.openGrabber(s.grabber).alongWith(new FloorPickUp(s.arm, s.wrist)));

    // eventMap.put("stow-arm", AutonHelper.openGrabber(s.grabber).alongWith(new PickUpFromGround(s.arm, s.wrist)));
    return AutonHelper.followWithEvents("pi-start-gamepiece1", eventMap,
     false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve, false);
  }

  public static Command twoGamePieceNoBumpPathPlaceCubeProxy(Subsystems s){
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    eventMap.put("raise-arm", new SetArmPose(s.arm, ArmPose.HIGH_NODE));
    return AutonHelper.followWithEvents("pi-gamepiece1-placecube", eventMap,
     false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve, false);
  }

  public static Command TwoGamePieceCenter(Subsystems s){
    return new SequentialCommandGroup(
      new InstantCommand(()->{
        SmartDashboard.putNumber("auton phase", 1);
      }),
      new SetArmPose(s.arm, ArmPose.HIGH_NODE),

      new InstantCommand(()->{
        s.swerve.setPose(new Pose2d(s.swerve.getPose().getX(), s.swerve.getPose().getY(), Rotation2d.fromDegrees(180)));
      }, s.swerve),

      new CustomProxy(()->{
        var destTrans = AutonHelper.getPathByName("startN4-gamepiece", defaultConstraints).getInitialHolonomicPose().getTranslation();
        return s.swerve.goTo(new Pose2d(destTrans, Rotation2d.fromDegrees(180)), Constants.AUTON.SLOW_CONSTRAINTS);
      }, s.swerve),

      AutonHelper.getPlaceConeCommand(s),

      new CustomProxy(()->{
        return twoGamePieceCenterPathPickUpCubeProxy(s);
      }),

      (AutonHelper.closeGrabber(s.grabber).andThen(new WaitCommand(0.25))).raceWith(new FloorPickUp(s.arm, s.wrist)),

      AutonHelper.stowAnd(s, new CustomProxy(()->{
        return twoGamePieceCenterPathDockProxy(s);
      })),
      new AutoBalance(s.swerve)

    ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
  }

  public static Command twoGamePieceCenterPathPickUpCubeProxy(Subsystems s){
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    eventMap.put("pickup", AutonHelper.openGrabber(s.grabber).alongWith(new FloorPickUp(s.arm, s.wrist)));

    return AutonHelper.followWithEvents("startN4-gamepiece", eventMap,
     false, Constants.AUTON.MID_CONSTRAINTS, s.swerve, false);
  }

  public static Command twoGamePieceCenterPathDockProxy(Subsystems s){
    HashMap<String, Command> eventMap = new HashMap<>();
    return AutonHelper.followWithEvents("center-gamepiece-dock", eventMap,
     false, Constants.AUTON.MID_CONSTRAINTS, s.swerve, false);
  }
  public static PathConstraints centerExploreThenDockConstraints = new PathConstraints(1.5,1.25);
  public static Command centerExploreThenDock(Subsystems s){
    return new SequentialCommandGroup(
        new SequentialCommandGroup(
        new InstantCommand(()->{
          SmartDashboard.putNumber("auton phase", 1);
        }),
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

      new InstantCommand(()->{
        s.swerve.setPose(new Pose2d(s.swerve.getPose().getX(), s.swerve.getPose().getY(), Rotation2d.fromDegrees(180)));
      }, s.swerve),

      new CustomProxy((

      )->{
        var destTrans = AutonHelper.getPathByName("macomb-startn4-explore", centerExploreThenDockConstraints).getInitialHolonomicPose().getTranslation();
        return s.swerve.goTo(new Pose2d(destTrans, Rotation2d.fromDegrees(180)), Constants.AUTON.SLOW_CONSTRAINTS);
      }, s.swerve)
      // AutonHelper.setInitPose(s, "InertN4-StartN4"),
      // AutonHelper.getCommandForPath("InertN4-StartN4", true, centerExploreThenDockConstraints, s.swerve)
      )
      .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY))
        ,
      new SequentialCommandGroup(
        AutonHelper.getPlaceConeCommand(s),

        new CustomProxy(()->{return centerExploreThenDockProxy(s);}),
        AutonHelper.getCommandForPath("macomb-explore-dock", false, centerExploreThenDockConstraints, s.swerve),
        new AutoBalance(s.swerve)
      )
      .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE))

    )
    // .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE))
    ;
  }

  public static Command centerExploreThenDockProxy(Subsystems s){
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    return AutonHelper.followWithEvents("macomb-startn4-explore", eventMap,
    false, centerExploreThenDockConstraints, s.swerve, false);
  }

  public static Command TwoGamePieceNoBumpNoVision(Subsystems s){
    return new SequentialCommandGroup(
      AutonHelper.setInitPose(s, "InertN1-StartN1"),

      new SetArmPose(s.arm, ArmPose.HIGH_NODE),

      AutonHelper.getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

      AutonHelper.getPlaceConeCommand(s),

      new CustomProxy(()->{
        return twoGamePieceNoBumpNoVisionPathPickUpCubeProxy(s);
      }),

      (AutonHelper.closeGrabber(s.grabber).andThen(new WaitCommand(0.25)))
      .raceWith(new FloorPickUp(s.arm, s.wrist))
      ,

      new CustomProxy(()->{
        return twoGamePieceNoBumpNoVisionPathPlaceCubeProxy(s);
      })

    ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE));
  }

  public static Command twoGamePieceNoBumpNoVisionPathPickUpCubeProxy(Subsystems s){
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    eventMap.put("pickup", AutonHelper.openGrabber(s.grabber).alongWith(new FloorPickUp(s.arm, s.wrist)));
    return AutonHelper.followWithEvents("startn1-gamepiece-novision", eventMap,
    false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve, false);
  }

  public static Command twoGamePieceNoBumpNoVisionPathPlaceCubeProxy(Subsystems s){
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    eventMap.put("raise-arm", new SetArmPose(s.arm, ArmPose.HIGH_NODE));
    return AutonHelper.followWithEvents("gamepiece-placecube-novision", eventMap,
    false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve, false);
  }
}
// format: on
