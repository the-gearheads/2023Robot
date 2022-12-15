package frc.robot.subsystems.drive.motors;

import edu.wpi.first.util.sendable.SendableBuilder;

public interface DriveMotor {
  public void setSpeed(double speed);
  public void setBrakeMode(boolean doBraking);

  public double getVelocity();
  public double getPosition();
  public void zeroEncoders();

  public void initSendable(SendableBuilder builder);
  public default void tickPID() {};

}
