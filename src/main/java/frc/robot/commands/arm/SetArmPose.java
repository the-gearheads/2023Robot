// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ControlMode;

public class SetArmPose extends CommandBase {
  private Arm arm;
  private ArmPose armPose;

  public enum ArmPose {
    FLOOR(0),
    HIGH_NODE(5),
    MID_NODE(3),
    LOW_NODE(3),
    FEEDER_STATION(1),
    INSIDE_ROBOT(1);

    double val;

    private ArmPose(double val){
      this.val=val;
    }

  }
  /** Creates a new SetArmPos. */
  public SetArmPose(Arm arm, ArmPose armPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.armPose = armPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.arm.controlMode = ControlMode.POS;
    this.arm.setGoal(this.armPose.val);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double goal = this.arm.getGoal();
    double currentPose = this.arm.getPosition();
    double poseError = goal-currentPose;
    poseError = Math.abs(poseError);
    return poseError < Constants.ARM.POSE_TOLERANCE;
  }
}
