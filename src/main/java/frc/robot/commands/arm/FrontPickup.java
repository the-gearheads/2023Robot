package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.wrist.SetWristPoseIndefinitely;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;

public class FrontPickup extends ParallelCommandGroup {

    public FrontPickup(Arm arm, Wrist wrist) {
        super(new SetArmPose(arm, ArmPose.FRONT_PICKUP), new SetWristPoseIndefinitely(wrist, 0));
        // Use addRequirements() here to declare subsystem dependencies.
    }
}
