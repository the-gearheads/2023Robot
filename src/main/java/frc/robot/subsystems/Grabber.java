// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GRABBER;

public class Grabber extends SubsystemBase {
  boolean isClosed = true;
  Solenoid closeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  Solenoid openSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
  PowerDistribution pdh = new PowerDistribution();
  Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  public Grabber() {
    compressor.enableAnalog(GRABBER.MIN_PRESSURE, GRABBER.MAX_PRESSURE);
    close();
  }

  public void close() {
    openSolenoid.set(false);
    closeSolenoid.set(true);
    isClosed = true;
  }

  public void open() {
    closeSolenoid.set(false);
    openSolenoid.set(true);
    isClosed = false;
  }

  public boolean isClosed() {
    return isClosed;
  }

  boolean wasLastEnabled = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var enabled = DriverStation.isEnabled();
    if (enabled && !wasLastEnabled) {
      // Turn spinny air pump cooling device on if enabled
      pdh.setSwitchableChannel(true);
    }
    if (!enabled && wasLastEnabled) {
      pdh.setSwitchableChannel(false);
    }

    wasLastEnabled = enabled;
    Logger.getInstance().recordOutput("Grabber/IsClosed", isClosed);
  }
}
