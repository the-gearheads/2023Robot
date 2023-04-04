package frc.robot.auton.paths;

import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.autoalign.Community;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.commands.wrist.FloorPickUp;
import frc.robot.subsystems.Subsystems;

@AutonAnnotation(name = "Two Cone", variants = {"N1", "N9"})
public class TwoCone extends AutonRoutine {
  private static PathConstraints TwoConeConstraints = Constants.AUTON.MID_CONSTRAINTS;
  @Override
  public CommandBase getCommand(Subsystems s, String v) {
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

      AutonHelper.closeGrabber(s.grabber),
      AutonHelper.followWithEvents(v + "_Gamepiece1-Cone2",
        Map.of("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),
                "raise-arm", new SetArmPose(s.arm, ArmPose.HIGH_NODE)
              ),
        false, TwoConeConstraints, s.swerve, false),

        AutonHelper.getPlaceConeCommand(s)

    ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
  }
}
