package frc.robot.subsystems.drive.motors;

public interface DriveMotor {
  public void setSpeed(double speed);
  public void setBrakeMode(boolean doBraking);

  public double getVelocity();
  public double getPosition();
  public void zeroEncoders();
}
