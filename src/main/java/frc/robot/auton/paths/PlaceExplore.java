package frc.robot.auton.paths;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.subsystems.Subsystems;

@AutonAnnotation(name = "Place Then Explore", variants = {"N1", "N9"})
public class PlaceExplore extends AutonRoutine {
    private static PathConstraints PlaceExploreConstraints = Constants.AUTON.MID_CONSTRAINTS;

    @Override
    public CommandBase getCommand(Subsystems s, String v) {
      return new SequentialCommandGroup(
        AutonHelper.setInitRot(s.swerve, v + "_Inert-Start"),

        AutonHelper.getCommandForPath(v + "_Inert-Start", true, PlaceExploreConstraints, s.swerve),
        AutonHelper.getPlaceConeCommand(s),
        AutonHelper.getCommandForPath(v + "_Start-Gamepiece1", false, PlaceExploreConstraints, s.swerve)
      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
    }
}
