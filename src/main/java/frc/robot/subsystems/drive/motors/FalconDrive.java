package frc.robot.subsystems.drive.motors;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DRIVE;

public class FalconDrive implements DriveMotor {

  private WPI_TalonFX motor;
  private DRIVE.DriveMotor drv = DRIVE.DRIVE_MOTOR;
  private PIDController pid;
  private double arbFF;
  private String path;


  public FalconDrive(int id, String path) {
    var pidf = drv.DRIVE_PIDF;
    this.motor = new WPI_TalonFX(id);
    this.pid = new PIDController(pidf[0], pidf[1], pidf[2]);
    this.arbFF = pidf[3];
    this.path = path;
    configure();
  }

  public void configure() {
    motor.configFactoryDefault();
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, drv.CURRENT_LIMIT, 0, 0));
  }

  private double nativeToMeters(double distance) {
    double ratioed = distance / DRIVE.DRIVE_GEAR_RATIO; // m -> rotations
    return ratioed * 8192.0;
  }

  @SuppressWarnings("unused")
  private double metersToNative(double distance) {
    // the above, but in reverse :)
    double unTalonified = distance / 8192.0;
    return unTalonified * DRIVE.DRIVE_GEAR_RATIO;
  }

  private double nativeVelToMeters(double velocity) {
    return nativeToMeters(velocity) / 10; // m/s -> m/100ms
  }

  public void setSpeed(double speed) {
    pid.setSetpoint(speed);
  }

  public void periodic() {
    // hope this works
    motor.set(pid.calculate(getPosition()) + pid.getSetpoint() * arbFF);
  }

  public void zeroEncoders() {
    motor.setSelectedSensorPosition(0);
  }

  public double getPosition() {
    return nativeToMeters(motor.getSelectedSensorPosition());
  }

  public double getVelocity() {
    return nativeVelToMeters(motor.getSelectedSensorVelocity());
  }

  public double getVelocitySetpoint() {
    return pid.getSetpoint();
  }

  public double getTemperature() {
    return motor.getTemperature();
  }

  public double getAppliedVolts() {
    return motor.getMotorOutputVoltage();
  }
}
