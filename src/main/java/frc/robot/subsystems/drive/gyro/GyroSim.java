// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GyroSim implements GyroIO {
  private Rotation2d rot;

  public GyroSim() {
    this.rot = new Rotation2d();
  }

  public Rotation2d getRotation2d() {
    return rot;
  }

  public void setRotation2d(Rotation2d newRotation2d) {
    rot = newRotation2d;
  }

  public double getRate() {
    return 0;
  }

  public void zeroYaw() {
    rot = new Rotation2d();
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.angleRadians = rot.getRadians();
    inputs.angleRadians = getRate();
  }
}
