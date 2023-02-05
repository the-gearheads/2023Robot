package frc.robot.subsystems.drive.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants.Drivetrain;
import frc.robot.util.SendableSparkMaxPID;

public class NeoDrive {
  private CANSparkMax max;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;
  private SendableSparkMaxPID sPid;

  private double setpoint;

  public NeoDrive(int id) {
    max = new CANSparkMax(id, MotorType.kBrushless);
    encoder = max.getEncoder();
    pid = max.getPIDController();
    sPid = new SendableSparkMaxPID(pid);
    configure();
  }

  private void configure() {
    max.restoreFactoryDefaults();
    max.setSmartCurrentLimit(Drivetrain.DRIVE_CURRENT_LIMIT);
    max.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(Drivetrain.DRIVE_POS_FACTOR);
    encoder.setVelocityConversionFactor(Drivetrain.DRIVE_VEL_FACTOR);

    // TODO: 1) check if this is too noisy 2) check if this impacts other robot code that deals with encoder velocities
    encoder.setMeasurementPeriod(10);

    pid.setFeedbackDevice(encoder);

    sPid.setP(Drivetrain.DRIVE_PIDF[0]);
    sPid.setI(Drivetrain.DRIVE_PIDF[1]);
    sPid.setD(Drivetrain.DRIVE_PIDF[2]);
    sPid.setFF(Drivetrain.DRIVE_PIDF[3]);

    // Probably the default
    pid.setOutputRange(-1, 1);

    /* Set periodic frame periods */
  
    /* Send our velocity more frequently */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    /* Don't have an analog encoder */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    /* Don't have an alternate encoder */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    /* Don't have a duty cycle encoder */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public void setSpeed(double speed) {
    setpoint = speed;
    pid.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  /* Please work */
  public void setVoltage(double volts) {
    SmartDashboard.putNumber("/Swerve/ManualVolts", volts);
    // pid.setReference(volts, CANSparkMax.ControlType.kVoltage);
    max.setVoltage(volts);
  }

  public void zeroEncoders() {
    encoder.setPosition(0);
  }

  public double getVelocitySetpoint() {
    return setpoint;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getAppliedVolts() {
    return max.getAppliedOutput() * max.getBusVoltage();
  }
}
