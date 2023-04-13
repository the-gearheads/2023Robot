package frc.robot.auton.paths;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.subsystems.Subsystems;

@AutonAnnotation(name = "Place Then Dock")
public class PlaceDock extends AutonRoutine {
    private static PathConstraints PlaceDockConstraints = Constants.AUTON.MID_CONSTRAINTS;

    @Override
    public CommandBase getCommand(Subsystems s, String v) {
      return new SequentialCommandGroup(
        AutonHelper.setInitRot(s.swerve, "N4_Inert-Start"),

        new SetArmPose(s.arm, ArmPose.HIGH_NODE),

        AutonHelper.getCommandForPath("N4_Inert-Start", true, PlaceDockConstraints, s.swerve),
        AutonHelper.getPlaceConeCommand(s),
        AutonHelper.getCommandForPath("N4_Start-PrepareDock", false, PlaceDockConstraints, s.swerve),
        new AutoBalance(s.swerve, s.grabber)
      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.NONE));
    }

}
