package frc.robot.subsystems.drive.motors;

import java.util.ArrayList;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants.DRIVE;
import frc.robot.util.RevConfigUtils;
import frc.robot.util.SendableSparkMaxPID;

public class NeoDrive {
  private CANSparkMax max;
  private RelativeEncoder encoder;
  private SparkMaxPIDController pid;
  private SendableSparkMaxPID sPid;

  private double setpoint;

  public NeoDrive(int id, String path) {
    max = new CANSparkMax(id, MotorType.kBrushless);
    encoder = max.getEncoder();
    pid = max.getPIDController();
    sPid = new SendableSparkMaxPID(pid);
    RevConfigUtils.configure(this::configure, path + "/Drive");
  }

  public ArrayList<REVLibError> configure() {
    ArrayList<REVLibError> e = new ArrayList<>();
    e.add(max.restoreFactoryDefaults());
    e.add(max.setSmartCurrentLimit(DRIVE.DRIVE_CURRENT_LIMIT));
    e.add(max.setIdleMode(IdleMode.kBrake));

    e.add(encoder.setPositionConversionFactor(DRIVE.DRIVE_POS_FACTOR));
    e.add(encoder.setVelocityConversionFactor(DRIVE.DRIVE_VEL_FACTOR));

    // TODO: 1) check if this is too noisy 2) check if this impacts other robot code that deals with encoder velocities
    e.add(encoder.setMeasurementPeriod(10));

    e.add(pid.setFeedbackDevice(encoder));

    e.add(sPid.setP(DRIVE.DRIVE_PIDF[0]));
    e.add(sPid.setI(DRIVE.DRIVE_PIDF[1]));
    e.add(sPid.setD(DRIVE.DRIVE_PIDF[2]));
    e.add(sPid.setFF(DRIVE.DRIVE_PIDF[3]));

    // Probably the default
    e.add(pid.setOutputRange(-1, 1));

    /* Set periodic frame periods */

    /* Send our velocity more frequently */
    e.add(max.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10));
    /* Don't have an analog encoder */
    e.add(max.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    /* Don't have an alternate encoder */
    e.add(max.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    /* Don't have a duty cycle encoder */
    e.add(max.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500));
    e.add(max.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500));

    return e;
  }

  public void setSpeed(double speed) {
    setpoint = speed;
    pid.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  /* Please work */
  public void setVoltage(double volts) {
    SmartDashboard.putNumber("Swerve/ManualVolts", volts);
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

  public double getTemperature() {
    return max.getMotorTemperature();
  }

  public double getAppliedVolts() {
    return max.getAppliedOutput() * max.getBusVoltage();
  }
}
