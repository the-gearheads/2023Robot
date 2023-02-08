package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechOperatorController implements OperatorController {

  Joystick joy;
  public LogitechOperatorController(int id) {
    joy = new Joystick(id);
  }

  @Override
  public double getArmAxis() {
    return -Controllers.deadband(joy.getY());
  }
}
