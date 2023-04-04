package frc.robot.auton.paths;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.AutoBalDriveToPivot;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.subsystems.Subsystems;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

@AutonAnnotation(name = "Explore Then Balance", variants = {"N4"})
public class ExploreBalance extends AutonRoutine {
    private static PathConstraints ExploreBalanceConstraints = Constants.AUTON.MID_CONSTRAINTS;

    @Override
    public CommandBase getCommand(Subsystems s, String v) {
      return new SequentialCommandGroup(
        AutonHelper.setInitRot(s.swerve, "N4_Inert-Start"),

        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        AutonHelper.getCommandForPath("N4_Inert-Start", true, ExploreBalanceConstraints, s.swerve, false),
        AutonHelper.getPlaceConeCommand(s),
        AutonHelper.getCommandForPath("N4_Start-Inert", false, ExploreBalanceConstraints, s.swerve, false),
        new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),

        AutonHelper.getCommandForPath("N4_Inert-Gamepiece1-NoTurn", false, ExploreBalanceConstraints, s.swerve),

        new WaitCommand(1),
        AutonHelper.getCommandForPath("N4_Gamepiece1-PrepareDock-NoTurn", false, ExploreBalanceConstraints, s.swerve),
        new AutoBalDriveToPivot(s.swerve).andThen(new AutoBalance(s.swerve))
      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
    }
}
