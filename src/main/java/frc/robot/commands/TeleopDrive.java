// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.AdditionalMathUtils;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private PIDController rotPIDController = new PIDController(1, 0, 0);
  private double angleSetPoint;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDrive(SwerveSubsystem subsystem) {
    swerveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    SmartDashboard.putBoolean("TeleopDrive/UseFieldRelative", false);
    SmartDashboard.putBoolean("TeleopDrive/ExponentialJoystickControl", false);
    SmartDashboard.putBoolean("Rotation PID", false);
    SmartDashboard.putNumber("Rotation PID kP", 1.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //swerveSubsystem.setPose(Constants.Drivetrain.zeroPos);
    angleSetPoint=swerveSubsystem.getPose().getRotation().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpd = Controllers.activeController.getXMoveAxis();
    var ySpd = Controllers.activeController.getYMoveAxis();
    var rotSpd = Controllers.activeController.getRotateAxis();

    boolean useExponentialJoystickControl = SmartDashboard.getBoolean("TeleopDrive/ExponentialJoystickControl", false);
    if(useExponentialJoystickControl){
      Pair<Double, Double> xyPair = AdditionalMathUtils.poseExp(xSpd, ySpd);
      xSpd=xyPair.getFirst();
      ySpd=xyPair.getSecond();
    }else{
      //Quadratic axis control
      xSpd *= Math.abs(xSpd);
      ySpd *= Math.abs(ySpd);
      rotSpd *= Math.abs(rotSpd);
    }

    xSpd *= Constants.Drivetrain.MAX_LIN_VEL;
    ySpd *= Constants.Drivetrain.MAX_LIN_VEL;
    rotSpd *= Constants.Drivetrain.MAX_ROT_VEL;

    //Make sure the robot maintains its heading when we aren't toggling the rotation axis
    if (SmartDashboard.getBoolean("Rotation PID", false)) {
      rotPIDController = new PIDController(SmartDashboard.getNumber("Rotation PID kP", 1), 0, 0);

      double gyroAngle = swerveSubsystem.gyro.getRotation2d().getRadians();
      if (MathUtil.applyDeadband(rotSpd, 1E-2) == 0) {
        rotSpd = -rotPIDController.calculate(angleSetPoint, gyroAngle);
      } else {
        angleSetPoint = gyroAngle;
      }
    }

    SmartDashboard.putNumber("TeleopDrive/xSpd", xSpd);
    SmartDashboard.putNumber("TeleopDrive/ySpd", ySpd);
    SmartDashboard.putNumber("TeleopDrive/rot", rotSpd);

    var speeds = new ChassisSpeeds(xSpd, ySpd, rotSpd);
    if (SmartDashboard.getBoolean("TeleopDrive/UseFieldRelative", true)) {
      swerveSubsystem.driveFieldRelative(speeds);
    } else {
      swerveSubsystem.drive(speeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
