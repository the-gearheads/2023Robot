// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    public Rotation2d getRotation2d();

    public void setRotation2d(Rotation2d newRotation2d);

    public double getRate();

    public void zeroYaw();
}
