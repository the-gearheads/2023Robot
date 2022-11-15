package frc.robot.subsystems.drive.motors;

public interface SteerMotor {
  /** Angle to set, in degrees */
  public void setAngle(double angle);
  public void setBrakeMode(boolean isBraking);

  public double getVelocity();
  public double getAngle();

  public void setPIDConstants(double kF, double kP, double kI, double kD);
  
  default public void zeroEncoders() {}
}
