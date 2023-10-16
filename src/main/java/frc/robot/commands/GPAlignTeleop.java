package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.gpdetect.GamePieceDetector;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;

public class GPAlignTeleop extends Command {
  Swerve swerve;
  GamePieceDetector gpdetect;
  Leds leds;
  /* Rough guesstimate, finetune */
  PIDController alignPid = new PIDController(3, 0, 0);
  public GPAlignTeleop(Swerve swerve, GamePieceDetector gpdetect, Leds leds) {
    this.swerve = swerve;
    this.gpdetect = gpdetect;
    this.leds = leds;
    this.alignPid.setTolerance(0.05);
    addRequirements(swerve, gpdetect);
  }

  @Override
  public void initialize() {
    gpdetect.enableDetection();
    alignPid.reset();
    leds.setState(LedState.ORANGE);
  }

  @Override
  public void execute() {
    if(gpdetect.isEnabled()) return; /* This probably takes time to happen so we're going to wait */
  
    /* Consider getting by closest to center instead */
    var mainDetection = gpdetect.getDetectionByArea();

    double vx = Controllers.driverController.getXMoveAxis() * DRIVE.MID_LIN_VEL;
    /* X axis is horizontal in camera space */
    double vy = MathUtil.clamp(alignPid.calculate(mainDetection.centerPoint.x, 0.5), -1, 1);
    double rot = Controllers.driverController.getRotateAxis() * DRIVE.MID_ROT_VEL;
    var speeds = new ChassisSpeeds(vx, vy, rot);

    swerve.drive(maintainHeading(speeds));
  }

  @Override
  public void end(boolean interrupted) {
    gpdetect.disableDetection();
    leds.resetState();
  }

  /* Stolen from TeleopDrive, but always enabled and no logging */
  private final PIDController rotPIDCnt = new PIDController(5d, 0d, 0d);
  private double angleGoal;
  private double lastRotSpdNotEqualZeroTimestamp = -1;

  /*Make sure the robot maintains its heading when we aren't toggling the rotation axis*/
  public ChassisSpeeds maintainHeading(ChassisSpeeds spds) {
    var rotSpd = spds.omegaRadiansPerSecond;
    var ctsGyroAngle = swerve.getCtsGyroRotWithOffset().getRadians();

    var rotSpdEqualZero = MathUtil.applyDeadband(rotSpd, 1E-2) == 0;

    if (!rotSpdEqualZero) {
      lastRotSpdNotEqualZeroTimestamp = Timer.getFPGATimestamp();
    }
    var currentTime = Timer.getFPGATimestamp();
    var waited = currentTime - lastRotSpdNotEqualZeroTimestamp > 0.1;

    SmartDashboard.putBoolean("waiting", !waited && rotSpdEqualZero);
    if (rotSpdEqualZero && waited) {
      rotSpd = rotPIDCnt.calculate(ctsGyroAngle, angleGoal);
      rotSpd = MathUtil.clamp(rotSpd, -2.5, 2.5);

      spds.omegaRadiansPerSecond = rotSpd;
    } else {
      angleGoal = ctsGyroAngle;
    }
    return spds;
  }

}
