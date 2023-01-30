package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleInputs {
    public double driveAppliedVolts;
    public double drivePosition;
    public double driveVelocity;
    public double driveVelocitySetpoint;

    public String description;

    public double steerAppliedVolts;
    public double steerVelocity;
    public double steerAngle;
    public double steerAngleSetpoint;
  }

  public default void setState(SwerveModuleState state) {}
  public default void zeroEncoders() {}
  public default void setPIDConstants(double kF, double kP, double kI, double kD) {}
  public default void setAngle(Rotation2d angle) {}
  public default void setVoltage(double volts) {} 
  public default void updateInputs(SwerveModuleInputs inputs) {}
  public default String getDescription() {return "";}
  public default String getPath() {return "";}
  public default int getId() {return 0;}
}