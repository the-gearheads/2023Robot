package frc.robot.auton.paths;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.subsystems.Subsystems;

@AutonAnnotation(name = "Place Then Explore", variants = {"N1", "N9"})
public class PlaceExplore extends AutonRoutine {
  private static PathConstraints PlaceExploreConstraints = Constants.AUTON.MID_CONSTRAINTS;

  @Override
  public Command getCommand(Subsystems s, String v) {
    return new SequentialCommandGroup(AutonHelper.setInitRot(s.swerve, v + "_Inert-Start"),

        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        AutonHelper.getCommandForPath(v + "_Inert-Start", true, PlaceExploreConstraints, s.swerve),
        AutonHelper.getPlaceConeCommand(s),
        AutonHelper.stowAnd(s,
            AutonHelper.getCommandForPath(v + "_Start-Explore", false, PlaceExploreConstraints, s.swerve)))
                .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE));
  }
}
