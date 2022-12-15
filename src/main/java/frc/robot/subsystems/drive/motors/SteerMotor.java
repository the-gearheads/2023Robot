package frc.robot.subsystems.drive.motors;

import edu.wpi.first.util.sendable.SendableBuilder;

public interface SteerMotor {
  /** Angle to set, in degrees */
  public void setAngle(double angle);
  public void setBrakeMode(boolean isBraking);

  public double getVelocity();
  public double getAngle();

  public void setPIDConstants(double kF, double kP, double kI, double kD);
  
  public void initSendable(SendableBuilder builder);

  default public void zeroEncoders() {}
  public default void tickPID() {};
}
