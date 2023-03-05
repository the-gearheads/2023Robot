// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlMode;

public class JoystickArmControl extends CommandBase {
  private Arm arm;

  /** Creates a new JoystickArmControl. */
  public JoystickArmControl(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // arm.setPoseGoal(arm.getPose());
    // arm.resetPIDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setControlMode(ArmControlMode.VEL);
    double pov = Controllers.operatorController.getPOVAngle();
    double axis = Controllers.operatorController.getArmAxis();
    double armPose = arm.getPose();

    SmartDashboard.putNumber("Arm/Axis", axis);
    SmartDashboard.putNumber("Arm/pov", pov);
    axis = Math.pow(axis, 3);

    double armvel = axis * Constants.ARM.VELOCITY;
    if (pov != -1) {
      double posvel = 0;
      if (pov > 90 && pov < 270) {
        posvel = 15;
      } else if (pov > 270 || pov < 90) {
        posvel = -15;
      }
      if (armPose < 90 && armPose > -90) {
        posvel *= -1;
      }
      armvel += posvel;
    }
    arm.setVelGoal(armvel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setVelGoal(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
