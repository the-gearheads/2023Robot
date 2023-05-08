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
import frc.robot.commands.drive.AutoBalance;
import frc.robot.commands.drive.autoalign.Community;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.vision.FuseVisionEstimate.ConfidenceStrat;
import frc.robot.commands.wrist.FloorPickUp;
import frc.robot.subsystems.Subsystems; 


@AutonAnnotation(name = "CENTER Two Cone")
public class CenterTwoCone extends AutonRoutine {
  private static PathConstraints TwoConeConstraints = Constants.AUTON.MID_CONSTRAINTS;
  private static PathConstraints centerTwoConeConstraints = new PathConstraints(1.1, 1);
  @Override
  public CommandBase getCommand(Subsystems s, String v) {
    
      return new SequentialCommandGroup(
        AutonHelper.setInitRot(s.swerve, "N4_Inert-Start"),
        new SetArmPose(s.arm, ArmPose.HIGH_NODE),
        AutonHelper.goToGridAlignment(s.swerve, Community.BLUE_GRID.centerGrid.leftCol.high, Community.RED_GRID.centerGrid.rightCol.high,
          Rotation2d.fromDegrees(180), centerTwoConeConstraints),

        AutonHelper.getPlaceConeCommand(s),

        AutonHelper.followWithEvents("N4_Start-Gamepiece1",
          Map.of("stow-arm", new SetArmPose(s.arm, ArmPose.INSIDE_ROBOT),
                  "pickup", new FloorPickUp(s.arm, s.wrist).alongWith(AutonHelper.openGrabber(s.grabber))
          ),
          false, centerTwoConeConstraints, s.swerve, false),

        AutonHelper.pickupCone(s),
        AutonHelper.stowAnd(s, 
          AutonHelper.getCommandForPath("N4_Gamepiece1-PrepareDock", false, centerTwoConeConstraints, s.swerve)
        ),
        new AutoBalance(s.swerve, s.grabber)
      ).raceWith(new FuseVisionEstimate(s.vision, ConfidenceStrat.ONLY_COMMUNITY));
  }
}
