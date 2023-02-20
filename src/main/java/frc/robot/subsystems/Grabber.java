// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GRABBER;

public class Grabber extends SubsystemBase {
  public enum GrabberState {
    OPEN, GRABBING_CUBE, // right soleniodeed is extended uwo
    GRABBING_CONE; // Both soliendoed extended
  }

  GrabberState grabberState = GrabberState.OPEN;
  Solenoid right = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Solenoid left = new Solenoid(PneumaticsModuleType.REVPH, 1);
  PowerDistribution pdh = new PowerDistribution();
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  public Grabber() {
    compressor.enableAnalog(GRABBER.MIN_PRESSURE, GRABBER.MAX_PRESSURE);
  }

  public void grabCone() {
    right.set(true);
    left.set(true);
    grabberState = GrabberState.GRABBING_CONE;
  }

  public void grabCube() {
    right.set(true);
    left.set(false);
    grabberState = GrabberState.GRABBING_CUBE;
  }

  public void open() {
    right.set(false);
    left.set(false);
    grabberState = GrabberState.OPEN;
  }

  public GrabberState getGrabberState() {
    return grabberState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Turn spinny air pump cooling device on if enabled
    pdh.setSwitchableChannel(DriverStation.isEnabled());
  }
}