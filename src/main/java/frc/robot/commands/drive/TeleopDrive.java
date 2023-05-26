// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
  private final PIDController rotPIDCnt = new PIDController(5d, 0d, 0d);
  private final SwerveRateLimit rateLimiter = new SwerveRateLimit();
  private double angleGoal;
  private double lastRotSpdNotEqualZeroTimestamp = -1;


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
    SmartDashboard.putBoolean("TeleopDrive/RateLimitDrive", false);
    SmartDashboard.putBoolean("TeleopDrive/rot pid/Turn On", true);
    SmartDashboard.putData("TeleopDrive/rot pid/PID Controller", rotPIDCnt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // swerve.setPose(new Pose2d());
    angleGoal = swerve.getCtsGyroRotWithOffset().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Controllers.driverController.getSetWheelXButton().getAsBoolean()) {
      swerve.setX();
      return;
    }

    var xSpd = Controllers.driverController.getXMoveAxis();
    var ySpd = Controllers.driverController.getYMoveAxis();
    var rotSpd = Controllers.driverController.getRotateAxis();

    var spds = new ChassisSpeeds(xSpd, ySpd, rotSpd);
    logSpds("Inputs", spds);

    spds = cube(spds);
    logSpds("Cubed", spds);

    if (SmartDashboard.getBoolean("TeleopDrive/RateLimitDrive", false)) {
      spds = rateLimit(spds);
      logSpds("Rate Limited", spds);
    }

    spds = scaleVel(spds);
    logSpds("Scaled", spds);

    cardinalBtns();
    spds = maintainHeading(spds);

    if (SmartDashboard.getBoolean("TeleopDrive/UseFieldRelative", true)) {
      swerve.driveFieldRelative(spds);
    } else {
      swerve.drive(spds);
    }
  }

  private void cardinalBtns() {
    double heading = Controllers.driverController.getSetHeadingPOV();
    var ctsGyroAngle = swerve.getCtsGyroRotWithOffset().getRadians();
    if (heading == -1)
      return;
    angleGoal = MoreMath.getClosestRad(ctsGyroAngle, Units.degreesToRadians(heading));
  }

  public ChassisSpeeds cube(ChassisSpeeds spds) {
    var xSpd = spds.vxMetersPerSecond;
    var ySpd = spds.vyMetersPerSecond;
    var rotSpd = spds.omegaRadiansPerSecond;

    //Cubic axis control (yes, i intentionally put the signum there in case we want to change to an even power)
    xSpd = Math.abs(Math.pow(xSpd, 3)) * Math.signum(xSpd);
    ySpd = Math.abs(Math.pow(ySpd, 3)) * Math.signum(ySpd);
    rotSpd = Math.abs(Math.pow(rotSpd, 3)) * Math.signum(rotSpd);

    return new ChassisSpeeds(xSpd, ySpd, rotSpd);
  }

  public ChassisSpeeds rateLimit(ChassisSpeeds spds) {
    if (SmartDashboard.getBoolean("TeleopDrive/RateLimitDrive", false)) {
      return rateLimiter.rateLimit(spds);
    }
    return spds;
  }

  public ChassisSpeeds scaleVel(ChassisSpeeds spds) {
    var lin_mult = Constants.DRIVE.MID_LIN_VEL;
    var rot_mult = Constants.DRIVE.MID_ROT_VEL;

    if (Controllers.driverController.LOW_SPEED().getAsBoolean()) {
      lin_mult = Constants.DRIVE.LOW_LIN_VEL;
      rot_mult = Constants.DRIVE.LOW_ROT_VEL;
    } else if (Controllers.driverController.HIGH_SPEED().getAsBoolean()) {
      lin_mult = Constants.DRIVE.HIGH_LIN_VEL;
      rot_mult = Constants.DRIVE.HIGH_ROT_VEL;
    } else if (Controllers.driverController.autoGrab().getAsBoolean()) {
      lin_mult = Constants.DRIVE.AUTO_GRAB_LIN_VEL;
      rot_mult = Constants.DRIVE.AUTO_GRAB_ROT_VEL;
    }
    spds.vxMetersPerSecond *= lin_mult;
    spds.vyMetersPerSecond *= lin_mult;
    spds.omegaRadiansPerSecond *= rot_mult;

    return spds;
  }

  /*Make sure the robot maintains its heading when we aren't toggling the rotation axis*/
  public ChassisSpeeds maintainHeading(ChassisSpeeds spds) {
    var rotSpd = spds.omegaRadiansPerSecond;
    var ctsGyroAngle = swerve.getCtsGyroRotWithOffset().getRadians();

    var runRotPid = SmartDashboard.getBoolean("TeleopDrive/rot pid/Turn On", false);
    var rotSpdEqualZero = MathUtil.applyDeadband(rotSpd, 1E-2) == 0;

    if (!rotSpdEqualZero) {
      lastRotSpdNotEqualZeroTimestamp = Timer.getFPGATimestamp();
    }
    var currentTime = Timer.getFPGATimestamp();
    var waited = currentTime - lastRotSpdNotEqualZeroTimestamp > 0.1;

    SmartDashboard.putBoolean("waiting", !waited && rotSpdEqualZero);
    if (runRotPid && rotSpdEqualZero && waited) {
      rotSpd = rotPIDCnt.calculate(ctsGyroAngle, angleGoal);
      Logger.getInstance().recordOutput("TeleopDrive/rot pid/rotSpd", rotSpd);
      rotSpd = MathUtil.clamp(rotSpd, -2.5, 2.5);
      rotSpd = MathUtil.applyDeadband(rotSpd, 0.1);
      Logger.getInstance().recordOutput("TeleopDrive/rot pid/clamped rotSpd", rotSpd);

      spds.omegaRadiansPerSecond = rotSpd;
      SmartDashboard.putBoolean("TeleopDrive/rot pid/running", true);
    } else {
      angleGoal = ctsGyroAngle;
      SmartDashboard.putBoolean("TeleopDrive/rot pid/running", false);
    }

    SmartDashboard.putNumber("TeleopDrive/rot pid/cts gyro angle", ctsGyroAngle);
    SmartDashboard.putNumber("TeleopDrive/rot pid/Goal", angleGoal);

    return spds;
  }

  public void logSpds(String subPath, ChassisSpeeds spds) {
    var xSpd = spds.vxMetersPerSecond;
    var ySpd = spds.vyMetersPerSecond;
    var rotSpd = spds.omegaRadiansPerSecond;

    Logger.getInstance().recordOutput("TeleopDrive/" + subPath + "/xSpd", xSpd);
    Logger.getInstance().recordOutput("TeleopDrive/" + subPath + "/ySpd", ySpd);
    Logger.getInstance().recordOutput("TeleopDrive/" + subPath + "/rotSpd", rotSpd);
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
