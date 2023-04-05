package frc.robot.auton.paths;

import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.autoalign.Community;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.commands.wrist.FloorPickUp;
import frc.robot.subsystems.Subsystems; 


@AutonAnnotation(name = "Two Cone", variants = {"N1", "N9", "N4"})
public class TwoCone extends AutonRoutine {
  private static PathConstraints TwoConeConstraints = Constants.AUTON.MID_CONSTRAINTS;
  private static PathConstraints centerTwoConeConstraints = new PathConstraints(1.1, 1);
  @Override
  public CommandBase getCommand(Subsystems s, String v) {
    if (v.equals("N1")) {
      return new SequentialCommandGroup(
        AutonHelper.setInitRot(s.swerve, v + "_Inert-Start"),

        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.leftGrid.leftCol.high, Community.RED_GRID.leftGrid.leftCol.high, 
          Rotation2d.fromDegrees(180), TwoConeConstraints),
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.followWithEvents(v + "_Start-Gamepiece1",
          Map.of("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),
                  "pickup", AutonHelper.openGrabber(s.grabber).alongWith(new FloorPickUp(s.arm, s.wrist))
                ),
          false, TwoConeConstraints, s.swerve, false
        ),
        
        AutonHelper.pickupCone(s),
        AutonHelper.followWithEvents(v + "_Gamepiece1-Cone2",
          Map.of("stow-arm", new StowArm(s.arm, s.wrist),
                  "raise-arm", new SetArmPose(s.arm, ArmPose.HIGH_NODE)
                ),
          false, TwoConeConstraints, s.swerve, false
        ),

        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.leftGrid.rightCol.high, Community.RED_GRID.leftGrid.rightCol.high, 
          Rotation2d.fromDegrees(180), TwoConeConstraints),
        AutonHelper.getPlaceConeCommand(s)
      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
    } else if (v.equals("N9")) {
      return new SequentialCommandGroup(
        AutonHelper.setInitRot(s.swerve, v + "_Inert-Start"),

        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.rightGrid.rightCol.high, Community.RED_GRID.rightGrid.rightCol.high, 
          Rotation2d.fromDegrees(180), TwoConeConstraints),
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.stowAnd(s, 
          AutonHelper.getCommandForPath("N9_Start-prebump", false, TwoConeConstraints, s.swerve)
        ),
        AutonHelper.getCommandForPath("prebump-overbump", false, Constants.AUTON.SLOW_CONSTRAINTS, s.swerve),
        AutonHelper.followWithEvents("overbump-gamepiece1",
          Map.of(
                  "pickup", AutonHelper.openGrabber(s.grabber).alongWith(new FloorPickUp(s.arm, s.wrist))
                ),
          false, TwoConeConstraints, s.swerve, false
        ),
        
        AutonHelper.pickupCone(s),
        AutonHelper.followWithEvents(v + "_Gamepiece1-Cone2",
          Map.of("stow-arm", new StowArm(s.arm, s.wrist),
                  "raise-arm", new SetArmPose(s.arm, ArmPose.HIGH_NODE)
                ),
          false, TwoConeConstraints, s.swerve, false
        ),
        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.rightGrid.leftCol.high, Community.RED_GRID.rightGrid.leftCol.high, 
        Rotation2d.fromDegrees(180), TwoConeConstraints),
        AutonHelper.getPlaceConeCommand(s)
      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
    } else {
      // center two cone
      return new SequentialCommandGroup(
        AutonHelper.setInitRot(s.swerve, v + "_Inert-Start"),
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.centerGrid.leftCol.high, Community.RED_GRID.centerGrid.leftCol.high,
          Rotation2d.fromDegrees(180), centerTwoConeConstraints),

        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.followWithEvents(v + "_Start-Gamepiece1",
          Map.of("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),
                  "pickup", new FloorPickUp(s.arm, s.wrist).alongWith(AutonHelper.openGrabber(s.grabber))
          ),
          false, centerTwoConeConstraints, s.swerve, false),

        AutonHelper.pickupCone(s),
        AutonHelper.stowAnd(s, 
          AutonHelper.getCommandForPath(v + "_Gamepiece1-PrepareDock", false, centerTwoConeConstraints, s.swerve)
        ),
        new AutoBalance(s.swerve)
      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
    }
  }
}
