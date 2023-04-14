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

  public double getPOVAngle() {
    return joy.getPOV();
  }

  public Trigger armGoToLowNode() {
    return new JoystickButton(joy, 5);
  }

  // public Trigger throwCube() {
  //   return new JoystickButton(joy, 8);
  // }

  public Trigger armGoToMidNode() {
    return new JoystickButton(joy, 6);
  }

  public Trigger armGoToHighNode() {
    return new JoystickButton(joy, 7);
  }

  public Trigger armGoToFeederStationNode() {
    return new JoystickButton(joy, 3);
  }

  public Trigger frontPickup() {
    return new JoystickButton(joy, 16);
  }

  public Trigger armGoToInsideRobotNode() {
    return new JoystickButton(joy, 4);
  }

  public Trigger setWristAlternatePose() {
    return new JoystickButton(joy, 2);
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

  public Trigger signalCube() {
    return new JoystickButton(joy, 9);

  }

  public Trigger signalCone() {
    return new JoystickButton(joy, 10);
  }

  public Trigger reconfigEVERYTHING() {
    return new JoystickButton(joy, 16);
  }

  @Override
  public Trigger autoGrab() {
    return new JoystickButton(joy, 8);
  }
}