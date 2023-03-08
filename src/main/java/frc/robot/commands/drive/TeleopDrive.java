// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MoreMath;
import frc.robot.util.SwerveRateLimit;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  private final Swerve swerve;
  private PIDController rotPIDCnt = new PIDController(3d, 0d, 0d);
  private SwerveRateLimit rateLimiter = new SwerveRateLimit();
  private double angleGoal;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public TeleopDrive(Swerve swerve) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    SmartDashboard.putBoolean("TeleopDrive/UseFieldRelative", true);
    SmartDashboard.putBoolean("TeleopDrive/ExponentialJoystickControl", false);
    SmartDashboard.putBoolean("TeleopDrive/RateLimitDrive", false);
    SmartDashboard.putBoolean("TeleopDrive/rot pid/Turn On", true);
    SmartDashboard.putData("TeleopDrive/rot pid/PID Controller", rotPIDCnt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // swerve.setPose(new Pose2d());
    var ctsGyroAngle=swerve.getContinuousGyroAngleRad();
    angleGoal = ctsGyroAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpd = Controllers.driverController.getXMoveAxis();
    var ySpd = Controllers.driverController.getYMoveAxis();
    var rotSpd = Controllers.driverController.getRotateAxis();

    Logger.getInstance().recordOutput("TeleopDrive/Deadbanded/xSpd", xSpd);
    Logger.getInstance().recordOutput("TeleopDrive/Deadbanded/ySpd", ySpd);
    Logger.getInstance().recordOutput("TeleopDrive/Deadbanded/rotSpd", rotSpd);

    if (Controllers.driverController.getSetWheelXButton().getAsBoolean()) {
      swerve.setX();
      return;
    }

    boolean useExponentialJoystickControl = SmartDashboard.getBoolean("TeleopDrive/ExponentialJoystickControl", false);
    if (useExponentialJoystickControl) {
      Pair<Double, Double> xyPair = MoreMath.poseExp(xSpd, ySpd);
      xSpd = xyPair.getFirst();
      ySpd = xyPair.getSecond();
    } else {
      //Cubic axis control (yes, i intentionally put the signum there in case we want to change to an even power)
      xSpd = Math.abs(Math.pow(xSpd, 3)) * Math.signum(xSpd);
      ySpd = Math.abs(Math.pow(ySpd, 3)) * Math.signum(ySpd);
      rotSpd = Math.abs(Math.pow(rotSpd, 3)) * Math.signum(rotSpd);
    }

    Logger.getInstance().recordOutput("TeleopDrive/Cubed/xSpd", xSpd);
    Logger.getInstance().recordOutput("TeleopDrive/Cubed/ySpd", ySpd);
    Logger.getInstance().recordOutput("TeleopDrive/Cubed/rotSpd", rotSpd);

    var limitedSpeeds = rateLimiter.rateLimit(new ChassisSpeeds(xSpd, ySpd, rotSpd));
    if (SmartDashboard.getBoolean("TeleopDrive/RateLimitDrive", false)) {
      xSpd = limitedSpeeds.vxMetersPerSecond;
      ySpd = limitedSpeeds.vyMetersPerSecond;
      rotSpd = limitedSpeeds.omegaRadiansPerSecond;
      Logger.getInstance().recordOutput("TeleopDrive/Rate Limited/xSpd", xSpd);
      Logger.getInstance().recordOutput("TeleopDrive/Rate Limited/ySpd", ySpd);
      Logger.getInstance().recordOutput("TeleopDrive/Rate Limited/rotSpd", rotSpd);
    }

    var lin_mult = Constants.DRIVE.MID_LIN_VEL;
    var rot_mult = Constants.DRIVE.MID_ROT_VEL;

    if (Controllers.driverController.LOW_SPEED().getAsBoolean()) {
      lin_mult = Constants.DRIVE.LOW_LIN_VEL;
      rot_mult = Constants.DRIVE.LOW_ROT_VEL;
    } else if (Controllers.driverController.HIGH_SPEED().getAsBoolean()) {
      lin_mult = Constants.DRIVE.HIGH_LIN_VEL;
      rot_mult = Constants.DRIVE.HIGH_ROT_VEL;
    }
    xSpd *= lin_mult;
    ySpd *= lin_mult;
    rotSpd *= rot_mult;

    Logger.getInstance().recordOutput("TeleopDrive/Scaled/xSpd", xSpd);
    Logger.getInstance().recordOutput("TeleopDrive/Scaled/ySpd", ySpd);
    Logger.getInstance().recordOutput("TeleopDrive/Scaled/rotSpd", rotSpd);

    rotSpd = applyRotPid(rotSpd);

    Logger.getInstance().recordOutput("TeleopDrive/Final/xSpd", xSpd);
    Logger.getInstance().recordOutput("TeleopDrive/Final/ySpd", ySpd);
    Logger.getInstance().recordOutput("TeleopDrive/Final/rotSpd", rotSpd);

    var speeds = new ChassisSpeeds(xSpd, ySpd, rotSpd);
    if (SmartDashboard.getBoolean("TeleopDrive/UseFieldRelative", true)) {
      swerve.driveFieldRelative(speeds);
    } else {
      swerve.drive(speeds);
      // swerve.driveFieldRelative(speeds);
    }
  }

  /*Make sure the robot maintains its heading when we aren't toggling the rotation axis*/
  public double applyRotPid(double rotSpd){
    var runRotPid = SmartDashboard.getBoolean("TeleopDrive/rot pid/Turn On", false);
    var rotSpdEqualZero = MathUtil.applyDeadband(rotSpd, 1E-2) == 0;

    var ctsGyroAngle=swerve.getContinuousGyroAngleRad();

    if (runRotPid && rotSpdEqualZero){
      rotSpd = rotPIDCnt.calculate(ctsGyroAngle, angleGoal);
      SmartDashboard.putString("TeleopDrive/rot pid/running", "pid");
    }else{
      angleGoal = ctsGyroAngle;
      SmartDashboard.putString("TeleopDrive/rot pid/running", "nothing");
    }

    SmartDashboard.putNumber("TeleopDrive/rot pid/cts gyro angle", ctsGyroAngle);
    SmartDashboard.putNumber("TeleopDrive/rot pid/Goal", angleGoal);
    Logger.getInstance().recordOutput("TeleopDrive/rot pid/rotSpd", rotSpd);

    rotSpd = MathUtil.clamp(rotSpd, -0.5, 0.5);
    Logger.getInstance().recordOutput("TeleopDrive/rot pid/clamped rotSpd", rotSpd);

    return rotSpd;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
