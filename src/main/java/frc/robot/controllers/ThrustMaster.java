package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ThrustMaster implements OperatorController {

  Joystick joy;

  public ThrustMaster(int id) {
    joy = new Joystick(id);
  }

  @Override
  public double getArmAxis() {
    return -Controllers.deadband(joy.getY());
  }

  public Trigger armGoToLowNode() {
    return new JoystickButton(joy, 5);
  }

  public Trigger armGoToMidNode() {
    return new JoystickButton(joy, 6);
  }

  public Trigger armGoToHighNode() {
    return new JoystickButton(joy, 7);
  }

  public Trigger armGoToFeederStationNode() {
    return new JoystickButton(joy, 3);
  }

  public Trigger armGoToInsideRobotNode() {
    return new JoystickButton(joy, 2);
  }

  public Trigger armGoToGroundPickUpNode() {
    return new JoystickButton(joy, 4);
  }

  public Trigger setWristAlternatePose() {
    return new JoystickButton(joy, 8);
  }

  public Trigger openGrabber() {
    return new JoystickButton(joy, 1);
  }

  public Trigger setArmByJoystick() {
    return new Trigger(() -> {
      if (Math.abs(getArmAxis()) > 0) {
        return true;
      }
      return false;
    });
  }
}
