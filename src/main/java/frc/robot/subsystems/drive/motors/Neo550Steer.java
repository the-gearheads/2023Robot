package frc.robot.subsystems.drive.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DRIVE;
import frc.robot.util.SendableSparkMaxPID;

public class Neo550Steer {

  private CANSparkMax max;
  private SparkMaxPIDController pid;
  private SendableSparkMaxPID sPid;
  private SparkMaxAbsoluteEncoder encoder;
  private String path;

  @SuppressWarnings("unused")
  private double dutyCycleOffset;
  private Rotation2d setpoint = new Rotation2d();

  public Neo550Steer(int id, double dutyCycleOffset, String path) {
    this.dutyCycleOffset = dutyCycleOffset;
    this.path = path;
    max = new CANSparkMax(id, MotorType.kBrushless);
    pid = max.getPIDController();
    sPid = new SendableSparkMaxPID(pid);
    encoder = max.getAbsoluteEncoder(Type.kDutyCycle);
    configure();
  }

  private void configure() {
    max.restoreFactoryDefaults();
    max.setSmartCurrentLimit(DRIVE.STEER_CURRENT_LIMIT);
    max.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(DRIVE.STEER_POS_FACTOR);
    encoder.setVelocityConversionFactor(DRIVE.STEER_VEL_FACTOR);

    // MaxSwerve steering encoder is inverted
    encoder.setInverted(true);

    pid.setFeedbackDevice(encoder);

    sPid.setP(DRIVE.STEER_PIDF[0]);
    sPid.setI(DRIVE.STEER_PIDF[1]);
    sPid.setD(DRIVE.STEER_PIDF[2]);
    sPid.setFF(DRIVE.STEER_PIDF[3]);

    SmartDashboard.putData(path + "/SteerPid", sPid);

    /* Probably the default */
    pid.setOutputRange(-1, 1);

    /* No need for angleMod360 with this */
    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMinInput(-Math.PI);
    /* I guess the rationale behind reusing pos factor instead of just putting 2pi here is that this lets us switch to degrees with only 1 change to the factors */
    pid.setPositionPIDWrappingMaxInput(Math.PI);

    /* Set periodic status intervals */

    /* Status 0 governs applied output, faults, and whether is a follower. We don't care about that super much, so we increase it */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    /* We don't care about our motor position, only what the encoder reads */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    /* Don't have an analog sensor */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* We -really- care about our duty cycle encoder readings though. THE DEFAULT WAS 200MS */
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    max.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    try {
      Thread.sleep((long) 40.0);
    } catch (Exception e) {
      e.printStackTrace();
    } ;
  }

  public void setAngle(Rotation2d angle) {
    setpoint = angle;
    pid.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition);
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
