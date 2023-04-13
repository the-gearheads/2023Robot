package frc.robot.auton;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.autoalign.Community;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.commands.wrist.FloorPickUp;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.CustomProxy;
import frc.robot.util.MoreMath;

// format: off
public class AutonPaths {
  /* Places cone in high node, goes to GP1 to pick up a cube, places it in low hybrid node
   * ASSUMPTIONS: Cone preloaded, arm already inside robot, in inert pos, pos constants correct
   */

  private static PathConstraints defaultConstraints = Constants.AUTON.MID_CONSTRAINTS;

  public static Command InertN4PlaceThenDock(Subsystems s) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      s.vision.setConfidenceStrat(ConfidenceStrat.NONE);
    }), AutonHelper.setInitPose(s, "InertN4-StartN4"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s,
            AutonHelper.getCommandForPath("StartN4-PrepareDock", false, Constants.AUTON.DOCK_CONSTRAINTS, s.swerve),
            new AutoBalance(s.swerve, s.grabber)));
  }

  public static Command InertN1Place(Subsystems s) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      s.vision.setConfidenceStrat(ConfidenceStrat.NONE);
    }), AutonHelper.setInitPose(s, "InertN4-StartN4"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN4-StartN4", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s));
  }

  public static Command InertN1PlaceThenExplore(Subsystems s) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      s.vision.setConfidenceStrat(ConfidenceStrat.NONE);
    }), AutonHelper.setInitPose(s, "InertN1-StartN1"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN1-StartN1", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN1-Explore", false, defaultConstraints, s.swerve)));
  }

  public static Command InertN9PlaceThenExplore(Subsystems s) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      s.vision.setConfidenceStrat(ConfidenceStrat.NONE);
    }), AutonHelper.setInitPose(s, "InertN9-StartN9"),

        // Move forward
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("InertN9-StartN9", true, defaultConstraints, s.swerve),

        // place game piece
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN9-Explore", false, defaultConstraints, s.swerve)));
  }

  public static Command TwoGamePieceNoBump(Subsystems s) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      SmartDashboard.putNumber("auton phase", 1);
      }), new InstantCommand(() -> {
        s.vision.setConfidenceStrat(ConfidenceStrat.ONLY_COMMUNITY);
      }), new SetArmPose(s.arm, ArmPose.HIGH_NODE), new InstantCommand(() -> {
        s.swerve.setPose(new Pose2d(s.swerve.getPose().getX(), s.swerve.getPose().getY(), Rotation2d.fromDegrees(180)));
      }, s.swerve),

      new CustomProxy(() -> {
        Translation2d destTrans;
        if (MoreMath.isBlue()) {
          destTrans = Community.BLUE_GRID.leftGrid.leftCol.high;
        } else {
          destTrans = Community.RED_GRID.rightGrid.rightCol.high;
        }
        return s.swerve.goTo(new Pose2d(destTrans, Rotation2d.fromDegrees(180)), Constants.AUTON.SLOW_CONSTRAINTS);
      }, s.swerve),

      AutonHelper.getPlaceConeCommand(s),

      new CustomProxy(() -> {
        return twoGamePieceNoBumpPathPickUpCubeProxy(s);
      }),

      (AutonHelper.closeGrabber(s.grabber).andThen(new WaitCommand(0.25))).raceWith(new FloorPickUp(s.arm, s.wrist)),

      new CustomProxy(() -> {
        return twoGamePieceNoBumpPathPlaceCubeProxy(s);
      }), AutonHelper.getPlaceConeCommand(s)

    );
  }

  public static Command twoGamePieceNoBumpPathPickUpCubeProxy(Subsystems s) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    eventMap.put("pickup", AutonHelper.openGrabber(s.grabber).alongWith(new FloorPickUp(s.arm, s.wrist)));

    // eventMap.put("stow-arm", AutonHelper.openGrabber(s.grabber).alongWith(new PickUpFromGround(s.arm, s.wrist)));
    return AutonHelper.followWithEvents("pi-start-gamepiece1", eventMap, false, Constants.AUTON.SLOW_CONSTRAINTS,
        s.swerve, false);
  }

  public static Command twoGamePieceNoBumpPathPlaceCubeProxy(Subsystems s) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    eventMap.put("raise-arm", new SetArmPose(s.arm, ArmPose.HIGH_NODE));
    return AutonHelper.followWithEvents("pi-gamepiece1-placecube", eventMap, false, Constants.AUTON.SLOW_CONSTRAINTS,
        s.swerve, false);
  }

  public static Command twoConeBump(Subsystems s) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      s.vision.setConfidenceStrat(ConfidenceStrat.NONE);
    }), new SetArmPose(s.arm, ArmPose.HIGH_NODE), new InstantCommand(() -> {
      s.swerve.setPose(new Pose2d(s.swerve.getPose().getX(), s.swerve.getPose().getY(), Rotation2d.fromDegrees(180)));
    }, s.swerve),

        new CustomProxy(() -> {
          Translation2d destTrans;
          if (MoreMath.isBlue()) {
            destTrans = Community.BLUE_GRID.rightGrid.rightCol.high;
          } else {
            destTrans = Community.RED_GRID.leftGrid.leftCol.high;
          }
          return s.swerve.goTo(new Pose2d(destTrans, Rotation2d.fromDegrees(180)), Constants.AUTON.SLOW_CONSTRAINTS);
        }, s.swerve),

        AutonHelper.getPlaceConeCommand(s),
        AutonHelper.stowAnd(s, AutonHelper.getCommandForPath("StartN9_prebump", false, defaultConstraints, s.swerve)),
        AutonHelper.getCommandForPath("prebump-overbump", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
        new SequentialCommandGroup(new WaitCommand(0.5),
            AutonHelper.openGrabber(s.grabber).alongWith(new FloorPickUp(s.arm, s.wrist)))
                .raceWith(AutonHelper.getCommandForPath("overbump-gamepiece1", false, defaultConstraints, s.swerve)),

        (AutonHelper.closeGrabber(s.grabber).andThen(new WaitCommand(0.25))).raceWith(new FloorPickUp(s.arm, s.wrist)),

        new CustomProxy(() -> {
          return twoconeBumpPlaceConeProxy(s);
        }),


        new CustomProxy(() -> {
          Translation2d destTrans;
          if (MoreMath.isBlue()) {
            destTrans = Community.BLUE_GRID.rightGrid.leftCol.high;
          } else {
            destTrans = Community.RED_GRID.leftGrid.rightCol.high;
          }
          return s.swerve.goTo(new Pose2d(destTrans, Rotation2d.fromDegrees(180)), Constants.AUTON.SLOW_CONSTRAINTS);
        }, s.swerve),

        AutonHelper.getPlaceConeCommand(s)

    );
  }

  public static Command twoconeBumpPlaceConeProxy(Subsystems s) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new StowArm(s.arm, s.wrist));
    eventMap.put("raise-arm", new SetArmPose(s.arm, ArmPose.HIGH_NODE));
    return AutonHelper.followWithEvents("StartN9_gamepiece1-InertN7", eventMap, false, defaultConstraints, s.swerve,
        false);
  }

  public static PathConstraints centerExploreThenDockSlowConstraints = new PathConstraints(1, 0.75);

  public static Command centerExploreThenDockSlow(Subsystems s) {
    return new SequentialCommandGroup(new SequentialCommandGroup(new InstantCommand(() -> {
      s.vision.setConfidenceStrat(ConfidenceStrat.ONLY_COMMUNITY);
    }), new InstantCommand(() -> {
      SmartDashboard.putNumber("auton phase", 1);
    }), new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        new InstantCommand(() -> {
          s.swerve
              .setPose(new Pose2d(s.swerve.getPose().getX(), s.swerve.getPose().getY(), Rotation2d.fromDegrees(180)));
        }, s.swerve),

        new CustomProxy((

        ) -> {
          // var path = AutonHelper.getPathByName("slow-startn4-explore", centerExploreThenDockSlowConstraints);
          // path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, DriverStation.getAlliance());
          // var destTrans = path.getInitialHolonomicPose().getTranslation();
          Translation2d destTrans;
          if (MoreMath.isBlue()) {
            destTrans = Community.BLUE_GRID.centerGrid.leftCol.high;
          } else {
            destTrans = Community.RED_GRID.centerGrid.rightCol.high;
          }
          return s.swerve.goTo(new Pose2d(destTrans, Rotation2d.fromDegrees(180)), Constants.AUTON.SLOW_CONSTRAINTS);
        }, s.swerve)
    // AutonHelper.setInitPose(s, "InertN4-StartN4"),
    // AutonHelper.getCommandForPath("InertN4-StartN4", true, centerExploreThenDockConstraints, s.swerve)
    ), new SequentialCommandGroup(new InstantCommand(() -> {
      s.vision.setConfidenceStrat(ConfidenceStrat.NONE);
    }), AutonHelper.getPlaceConeCommand(s),

        new CustomProxy(() -> {
          return centerExploreThenDockSlowProxy(s);
        }), AutonHelper.getCommandForPath("slow-explore-dock", false, centerExploreThenDockSlowConstraints, s.swerve),
        new AutoBalance(s.swerve, s.grabber))

    );
  }

  public static Command centerExploreThenDockSlowProxy(Subsystems s) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
    return AutonHelper.followWithEvents("slow-startn4-explore", eventMap, false, centerExploreThenDockSlowConstraints,
        s.swerve, false);
  }

  public static Command center2Cone(Subsystems s){
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
          s.vision.setConfidenceStrat(ConfidenceStrat.ONLY_COMMUNITY);
        }), 
      new SetArmPose(s.arm, ArmPose.HIGH_NODE),
      new InstantCommand(() -> {
        s.swerve
            .setPose(new Pose2d(s.swerve.getPose().getX(), s.swerve.getPose().getY(), Rotation2d.fromDegrees(180)));
      }, s.swerve),

      new CustomProxy((
      ) -> {
        Translation2d destTrans;
        if (MoreMath.isBlue()) {
          destTrans = Community.BLUE_GRID.centerGrid.leftCol.high;
        } else {
          destTrans = Community.RED_GRID.centerGrid.rightCol.high;
        }
        return s.swerve.goTo(new Pose2d(destTrans, Rotation2d.fromDegrees(180)), Constants.AUTON.SLOW_CONSTRAINTS);
      }, s.swerve),
      
      new InstantCommand(() -> {
        s.vision.setConfidenceStrat(ConfidenceStrat.NONE);
      }), AutonHelper.getPlaceConeCommand(s),

      new CustomProxy(() -> {
        return center2ConeFirstProxy(s);
      }), 
      (AutonHelper.closeGrabber(s.grabber).andThen(new WaitCommand(0.25))).raceWith(new FloorPickUp(s.arm, s.wrist)),
      new CustomProxy(() -> {
        return center2ConeSecondProxy(s);
      }),
      new AutoBalance(s.swerve, s.grabber)); 
    }
      public static Command center2ConeFirstProxy(Subsystems s) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
        eventMap.put("pickup", new FloorPickUp(s.arm, s.wrist).alongWith(AutonHelper.openGrabber(s.grabber)));
        return AutonHelper.followWithEvents("center2cone-startn4-pickup", eventMap, false, centerExploreThenDockSlowConstraints,
            s.swerve, false);
      }

      public static Command center2ConeSecondProxy(Subsystems s) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT));
        return AutonHelper.followWithEvents("center2cone-pickup-dock", eventMap, false, centerExploreThenDockSlowConstraints,
            s.swerve, false);
      }
}
// format: on
