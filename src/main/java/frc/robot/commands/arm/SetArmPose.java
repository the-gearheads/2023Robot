// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlMode;

public class SetArmPose extends Command {
  private Arm arm;
  private double armPoseVal;

  public enum ArmPose {//-8.2 for front pickup -184 for feederstation
    FRONT_PICKUP(-6.2), FLOOR(-74), HIGH_NODE(4), MID_NODE(-15.46), LOW_NODE(-61.87), FEEDER_STATION(
        -187), INSIDE_ROBOT(-110), VARIABLE(0);

    public double val;

    private ArmPose(double val) {
      this.val = val;
    }

    private void setGoal(double val) {
      this.val = val;
    }

  }

  /**
   * Sets arm to a given pose
   * 
   * @param armPose Desired arm pose
   */
  public SetArmPose(Arm arm, ArmPose armPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.armPoseVal = armPose.val;
    addRequirements(arm);
  }

  public SetArmPose(Arm arm, double armPoseVal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.armPoseVal = armPoseVal;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setControlMode(ArmControlMode.POS);
    arm.setPoseGoal(armPoseVal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Controllers.operatorController.setArmByJoystick().getAsBoolean()) {
      return true;
    }

    double goal = armPoseVal;
    double currentPose = arm.getPose();
    double poseError = goal - currentPose;
    poseError = Math.abs(poseError);
    return poseError < Constants.ARM.POSE_TOLERANCE;
  }
}
