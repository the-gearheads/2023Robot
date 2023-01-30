package frc.robot.subsystems.drive.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Drivetrain;
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
    max.setSmartCurrentLimit(Drivetrain.STEER_CURRENT_LIMIT);
    max.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(Drivetrain.STEER_POS_FACTOR);
    encoder.setVelocityConversionFactor(Drivetrain.STEER_VEL_FACTOR);

    // MaxSwerve steering encoder is inverted
    encoder.setInverted(true);

    pid.setFeedbackDevice(encoder);

    sPid.setP(Drivetrain.STEER_PIDF[0]);
    sPid.setI(Drivetrain.STEER_PIDF[1]);
    sPid.setD(Drivetrain.STEER_PIDF[2]);
    sPid.setFF(Drivetrain.STEER_PIDF[3]);

    SmartDashboard.putData(path + "/SteerPid", sPid);

    /* Probably the default */
    pid.setOutputRange(-1, 1);

    /* No need for angleMod360 with this */
    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMinInput(-Math.PI);
    /* I guess the rationale behind reusing pos factor instead of just putting 2pi here is that this lets us switch to degrees with only 1 change to the factors */
    pid.setPositionPIDWrappingMaxInput(Math.PI);
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
