package frc.robot.subsystems.drive.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Drivetrain;
import frc.robot.util.SendableSparkMaxPID;

public class NeoDrive {
  CANSparkMax max;
  RelativeEncoder encoder;
  SparkMaxPIDController pid;
  SendableSparkMaxPID sPid;

  double setpoint;

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

    pid.setFeedbackDevice(encoder);

    sPid.setP(Drivetrain.DRIVE_PIDF[0]);
    sPid.setI(Drivetrain.DRIVE_PIDF[1]);
    sPid.setD(Drivetrain.DRIVE_PIDF[2]);
    sPid.setFF(Drivetrain.DRIVE_PIDF[3]);

    // Probably the default
    pid.setOutputRange(-1, 1);
  }

  public void setSpeed(double speed) {
    setpoint = speed;
    pid.setReference(speed, CANSparkMax.ControlType.kVelocity);
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
