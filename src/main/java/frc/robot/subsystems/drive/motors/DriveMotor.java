package frc.robot.subsystems.drive.motors;

public interface DriveMotor {
  public void configure();
  public void setSpeed(double speed);
  public default void setVoltage(double volts) {};
  public default void periodic() {};
  public void zeroEncoders();
  public double getVelocitySetpoint();
  public double getPosition();
  public double getVelocity();
  public double getTemperature();
  public double getAppliedVolts();
}
