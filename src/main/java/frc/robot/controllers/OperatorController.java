package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {
  public default double getArmAxis() {
    return 0.0;
  }
  public default Trigger armGoTo1ndNode() {
    return new Trigger(()->false);
  }
  public default Trigger armGoTo2ndNode() {
    return new Trigger(()->false);
  }
  public default Trigger armGoTo3ndNode() {
    return new Trigger(()->false);
  }
  public default Trigger armGoToFeederStationNode() {
    return new Trigger(()->false);
  }
  public default Trigger armGoToInsideRobotNode() {
    return new Trigger(()->false);
  }
  public default Trigger armGoToGroundPickUpNode() {
    return new Trigger(()->false);
  }
  public default Trigger setArmByJoystick() {
    return new Trigger(()->{
      return false;
    });
  }
}
