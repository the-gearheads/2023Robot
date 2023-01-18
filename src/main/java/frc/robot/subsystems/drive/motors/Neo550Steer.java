package frc.robot.subsystems.drive.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.Drivetrain;

public class Neo550Steer {

  CANSparkMax max;
  SparkMaxPIDController pid;
  SparkMaxAbsoluteEncoder encoder;

  double dutyCycleOffset;
  Rotation2d setpoint;

  public Neo550Steer(int id, double dutyCycleOffset) {
    this.dutyCycleOffset = dutyCycleOffset;
    max = new CANSparkMax(id, MotorType.kBrushless);
    pid = max.getPIDController();
    encoder = max.getAbsoluteEncoder(Type.kDutyCycle);
    configure();
  }
  
  private void configure() {
    max.restoreFactoryDefaults();
    max.setSmartCurrentLimit(Drivetrain.STEER_CURRENT_LIMIT);
    max.setIdleMode(IdleMode.kBrake);

    encoder.setZeroOffset(dutyCycleOffset);
    encoder.setPositionConversionFactor(Drivetrain.STEER_POS_FACTOR);
    encoder.setVelocityConversionFactor(Drivetrain.STEER_VEL_FACTOR);

    pid.setFeedbackDevice(encoder);

    pid.setP(Drivetrain.STEER_PIDF[0]);
    pid.setI(Drivetrain.STEER_PIDF[1]);
    pid.setD(Drivetrain.STEER_PIDF[2]);
    pid.setFF(Drivetrain.STEER_PIDF[3]);

    /* Probably the default */
    pid.setOutputRange(-1, 1);

    /* No need for angleMod360 with this */
    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMinInput(0);
    /* I guess the rationale behind reusing pos factor instead of just putting 2pi here is that this lets us switch to degrees with only 1 change to the factors */
    pid.setPositionPIDWrappingMaxInput(Drivetrain.STEER_POS_FACTOR);
  }

  public void setAngle(Rotation2d angle) {
    setpoint = angle; 
    pid.setReference(angle.getRadians(), ControlType.kPosition);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(encoder.getPosition());
  }

  public Rotation2d getAngleSetpoint() {
    return setpoint;
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getAppliedVolts() {
    return max.getAppliedOutput() * max.getBusVoltage();
  }

  public void updatePIDConstants(double P, double I, double D, double F) {
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
    pid.setFF(F);
  }
}