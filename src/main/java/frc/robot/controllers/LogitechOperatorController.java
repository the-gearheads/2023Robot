package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechOperatorController implements OperatorController {

  Joystick joy;
  public LogitechOperatorController(int id) {
    joy = new Joystick(id);
  }

  @Override
  public double getArmAxis() {
    return -Controllers.deadband(joy.getY());
  }

  public Trigger armGoTo1ndNode() {
    return new JoystickButton(joy, 6);
  }
  public Trigger armGoTo2ndNode() {
    return new JoystickButton(joy, 5);
  }
  public Trigger armGoTo3ndNode() {
    return new JoystickButton(joy, 1);
  }
  public Trigger armGoToFeederStationNode() {
    return new JoystickButton(joy, 4);
  }
  public Trigger armGoToInsideRobotNode() {
    return new JoystickButton(joy, 3);
  }
  public Trigger armGoToGroundPickUpNode() {
    return new JoystickButton(joy, 2);
  }
  public Trigger setArmByJoystick() {
    return new Trigger(()->{
      if(Math.abs(getArmAxis())>0.2){
        if(Math.random()>0.5){
          return true;
        }
      }
      return false;
    });
  }
}
