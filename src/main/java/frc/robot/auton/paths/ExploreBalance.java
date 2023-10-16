package frc.robot.auton.paths;

import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonHelper;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.autoalign.Community;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.subsystems.Subsystems;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@AutonAnnotation(name = "CENTER Explore Then Balance")
public class ExploreBalance extends AutonRoutine {
  private static PathConstraints ExploreBalanceConstraints = Constants.AUTON.MID_CONSTRAINTS;
  public static PathConstraints centerExploreThenDockSlowConstraints = new PathConstraints(1, 0.75);

  @Override
  public Command getCommand(Subsystems s, String v) {
    return new SequentialCommandGroup(AutonHelper.setInitRot(s.swerve, "N4_Inert-Start"),

        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.centerGrid.leftCol.high,
            Community.RED_GRID.centerGrid.rightCol.high, Rotation2d.fromDegrees(180), ExploreBalanceConstraints),
        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.followWithEvents("N4_Start-Explore-slow",
            Map.of("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT)), false,
            centerExploreThenDockSlowConstraints, s.swerve, false),
        // AutonHelper.getCommandForPath("N4_Start-Inert", false, ExploreBalanceConstraints, s.swerve, false),
        // new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),

        AutonHelper.getCommandForPath("N4_Explore-PrepareDock", false, centerExploreThenDockSlowConstraints, s.swerve),

        new AutoBalance(s.swerve, s.grabber))
            .raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
  }
}
