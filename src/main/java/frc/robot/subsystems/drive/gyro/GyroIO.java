// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {

  @AutoLog
  public class GyroIOInputs {
    public double angleRadians;
    public double angleRate;
    public double pitchDegrees;
    public double rollDegrees;
    public double linearAccelY;
    public double linearVelY;
    public double linearAccelZ;
    public double linearVelZ;
  }

  public default void setRotation2d(Rotation2d newRotation2d) {};

  public default void setRate(double omegaRadiansPerSecond) {};

  public default void zeroYaw() {};

  public default void updateInputs(GyroIOInputs inputs) {};
}
