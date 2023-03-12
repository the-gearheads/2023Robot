package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SegaOperatorController implements OperatorController {

  Joystick joy;

  public SegaOperatorController(int id) {
    joy = new Joystick(id);
  }

  public Trigger armGoToLowNode() {
    return new JoystickButton(joy, 5);
  }

  public Trigger armGoToMidNode() {
    return new JoystickButton(joy, 3);
  }

  public Trigger armGoToHighNode() {
    return new JoystickButton(joy, 2);
  }

  public Trigger armGoToFeederStationNode() {
    return new JoystickButton(joy, 4);
  }

  public Trigger armGoToInsideRobotNode() {
    return new JoystickButton(joy, 1);
  }

  public Trigger armGoToGroundPickUpNode() {
    return new JoystickButton(joy, 6);
  }

  public Trigger setWristAlternatePose() {
    return new JoystickButton(joy, 7);
  }

  public Trigger openGrabber() {
    return new JoystickButton(joy, 8);
  }

  private Trigger downDPAD() {
    return new JoystickButton(joy, 13);
  }

  private Trigger upDPAD() {
    return new JoystickButton(joy, 12);
  }

  public Double getPOV() {
    if (upDPAD().getAsBoolean()) {
      return 0.0;
    } else if (downDPAD().getAsBoolean()) {
      return 180.0;
    } else {
      return -1.0;
    }
  }
}
