package frc.robot.auton.paths;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.subsystems.Subsystems;

@AutonAnnotation(name = "Place Then Explore", variants = {"N4"})
public class PlaceDock extends AutonRoutine {
    private static PathConstraints PlaceDockConstraints = Constants.AUTON.MID_CONSTRAINTS;

    @Override
    public CommandBase getCommand(Subsystems s, String v) {
      return new SequentialCommandGroup(
        AutonHelper.setInitRot(s.swerve, v + "_Inert-Start"),

        AutonHelper.getCommandForPath(v + "_Inert-Start", true, PlaceDockConstraints, s.swerve),
        AutonHelper.getPlaceConeCommand(s),
        AutonHelper.getCommandForPath(v + "_Start-PrepareDock", false, PlaceDockConstraints, s.swerve),
        new AutoBalance(s.swerve)
      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
    }

}
