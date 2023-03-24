package frc.robot.subsystems.leds;


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.LEDS;

public class Leds extends SubsystemBase {
  /** Creates a new LEDS. d */
  private AddressableLED ledStrip;
  private AddressableLEDSim ledSim;

  // one buffer for all the live shit, one for no lights
  public AddressableLEDBuffer buffer;

  // LED States
  public LedState state;
  // Assuming that both strips are the same length - therefore we can use one buffer

  private LedState defaultState = LedState.RAINBOW;

  public Leds() {
    buffer = new AddressableLEDBuffer(LEDS.LENGTH);

    ledStrip = new AddressableLED(LEDS.PORT);
    ledStrip.setLength(buffer.getLength());

    ledSim = new AddressableLEDSim(ledStrip);
    ledSim.setOutputPort(LEDS.PORT);

    state = defaultState;

    // send voltages to the strips
    startLED();
  }

  public void setState(LedState newState) {
    this.state = newState;
  }

  public CommandBase getSetStateCommand(LedState newState) {
    return new InstantCommand(() -> setState(newState), this);
  }

  public CommandBase setStateForTimeCommand(LedState newState, double waitSecs) {
    return getSetStateCommand(newState)
      .andThen(new WaitCommand(waitSecs))
      .andThen(getSetStateCommand(defaultState));
  }

  // method to flush both strips
  public void startLED() {
    ledStrip.start();
  }

  @Override
  public void periodic() {
    this.state.updateBuffer(buffer);

    /* Should overwrite what the above wrote to the buffer for the out-of-time warning */
    if(DriverStation.isTeleop()) {
      double timeRemaining = Robot.matchTime;
      if (timeRemaining >= 57.0 && timeRemaining < 60.0) {
        LedState.FLASH_YELLOW.updateBuffer(buffer);
      } else if (timeRemaining >= 27.0 && timeRemaining < 30.0) {
        LedState.FLASH_RED.updateBuffer(buffer);
      }
    }
    this.ledStrip.setData(buffer);
  }
}

