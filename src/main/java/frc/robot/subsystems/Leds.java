package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Leds extends SubsystemBase {
  /** Creates a new LEDS. */
  private AddressableLED ledStrip;
  private AddressableLEDSim ledSim;

  // one buffer for all the live shit, one for no lights
  public AddressableLEDBuffer yellowBuffer;
  public AddressableLEDBuffer whiteBuffer;
  public AddressableLEDBuffer greenBuffer;

  // Assuming that both strips are the same length - therefore we can use one buffer
  private int length = 0;

  public Leds(int port, int stripLength) {
    ledStrip = new AddressableLED(port);
    length = stripLength;
    ledStrip.setLength(length);

    ledSim = new AddressableLEDSim(ledStrip);

    // initialize buffers
    greenBuffer = new AddressableLEDBuffer(length);
    whiteBuffer = new AddressableLEDBuffer(length);
    yellowBuffer = new AddressableLEDBuffer(length);

    fillYellowBuffer();
    fillWhiteBuffer();
    fillGreenBuffer();

    // send voltages to the strips
    startStrips();

    updateStrips(greenBuffer);
  }

  // update both strips to a new buffer
  public void updateStrips(AddressableLEDBuffer buffer) {
    ledStrip.setData(buffer);
  }

  // method to flush both strips
  public void startStrips() {
    ledStrip.start();
  }

  // fill null buffer with (0, 0, 0)
  public void fillYellowBuffer() {
    for (int i = 0; i < yellowBuffer.getLength(); i++) {
      yellowBuffer.setRGB(i, 54, 237, 54);
    }
  }

  public void fillGreenBuffer() {
    for (int i = 0; i < greenBuffer.getLength(); i++) {
      greenBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public void fillWhiteBuffer() {
    for (int i = 0; i < whiteBuffer.getLength(); i++) {
      whiteBuffer.setRGB(i, 233, 233, 233);
    }
  }

  public void clearBuffer(AddressableLEDBuffer buffer) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
  }

  @Override
  public void periodic() {

  }
}

