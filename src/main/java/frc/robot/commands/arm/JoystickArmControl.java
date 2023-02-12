// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmControlMode;

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
    this.arm.controlMode = ArmControlMode.VEL;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.arm.controlMode = ArmControlMode.VEL;
    double axis = Controllers.operatorController.getArmAxis();
    SmartDashboard.putNumber("Arm Axis", axis);
    axis *= Math.abs(axis);
    double armvel = axis * Constants.ARM.VELOCITY;
    this.arm.setGoal(armvel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.arm.setGoal(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
