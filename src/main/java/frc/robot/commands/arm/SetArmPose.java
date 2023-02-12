// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmControlMode;

public class SetArmPose extends CommandBase {
  private Arm arm;
  private ArmPose armPose;

  public enum ArmPose {
    FLOOR(-30),
    HIGH_NODE(-195),
    MID_NODE(-180),
    LOW_NODE(-165),
    FEEDER_STATION(0),
    INSIDE_ROBOT(-90);

    double val;

    private ArmPose(double val){
      this.val=Units.degreesToRadians(val);
    }

  }
  /** Creates a new SetArmPos. */
  public SetArmPose(Arm arm, ArmPose armPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.armPose = armPose;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.arm.controlMode = ArmControlMode.POS;
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
