package frc.robot.auton.paths;

import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.autoalign.Community;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.commands.wrist.FloorPickUp;
import frc.robot.subsystems.Subsystems;


@AutonAnnotation(name = "NO BUMP Two Cone")
public class NoBumpTwoCone extends AutonRoutine {
  private static PathConstraints TwoConeConstraints = Constants.AUTON.MID_CONSTRAINTS;

  @Override
  public Command getCommand(Subsystems s, String v) {
    return new SequentialCommandGroup(AutonHelper.setInitRot(s.swerve, "N1_Inert-Start"),

        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.leftGrid.leftCol.high,
            Community.RED_GRID.rightGrid.rightCol.high, Rotation2d.fromDegrees(180), TwoConeConstraints),
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.followWithEvents("N1_Start-Gamepiece1",
            Map.of("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT), "pickup",
                AutonHelper.openGrabber(s.grabber).alongWith(new FloorPickUp(s.arm, s.wrist))),
            false, TwoConeConstraints, s.swerve, false),

        AutonHelper.pickupCone(s),
        AutonHelper.followWithEvents("N1_Gamepiece1-Cone2",
            Map.of("stow-arm", new StowArm(s.arm, s.wrist), "raise-arm", new SetArmPose(s.arm, ArmPose.HIGH_NODE)),
            false, TwoConeConstraints, s.swerve, false),
        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.leftGrid.rightCol.high,
            Community.RED_GRID.rightGrid.leftCol.high, Rotation2d.fromDegrees(180), TwoConeConstraints),
        AutonHelper.getPlaceConeCommand(s)).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
  }
}
