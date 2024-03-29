// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

/** Add your docs here. */
public class ThrowState {
  public double wristPose;
  public double armSpeed;
  public double armPose;

  public ThrowState(double armPose, double armSpeed, double wristPose) {
    this.armPose = armPose;
    this.armSpeed = armSpeed;
    this.wristPose = wristPose;
  }

  public enum ThrowPhase {
    INIT, PRE_RELEASE, RELEASE, WAIT, FINAL, END;
  }
}
