package frc.robot.subsystems.leds;


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDS;

public class Leds extends SubsystemBase {
  /** Creates a new LEDS. */
  private AddressableLED ledStrip;
  private AddressableLEDSim ledSim;

  // one buffer for all the live shit, one for no lights
  public AddressableLEDBuffer buffer;

  // LED States
  public LedState state;
  // Assuming that both strips are the same length - therefore we can use one buffer

  public Leds() {
    buffer = new AddressableLEDBuffer(LEDS.length);

    ledStrip = new AddressableLED(LEDS.port);
    ledStrip.setLength(buffer.getLength());

    ledSim = new AddressableLEDSim(ledStrip);

    state = LedState.GREEN;

    // send voltages to the strips
    startLED();
  }

  public void setState(LedState newState) {
    this.state = newState;
  }

  // method to flush both strips
  public void startLED() {
    ledStrip.start();
  }

  @Override
  public void periodic() {
    this.state.updateBuffer(buffer);
    this.ledStrip.setData(buffer);
  }
}

