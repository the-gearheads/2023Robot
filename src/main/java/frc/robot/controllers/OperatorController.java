package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {
  public default double getArmAxis() {
    return 0.0;
  }

  public default Trigger armGoToLowNode() {
    return new Trigger(() -> false);
  }

  public default Trigger armGoToMidNode() {
    return new Trigger(() -> false);
  }

  public default Trigger armGoToHighNode() {
    return new Trigger(() -> false);
  }

  public default Trigger armGoToFeederStationNode() {
    return new Trigger(() -> false);
  }

  public default Trigger armGoToInsideRobotNode() {
    return new Trigger(() -> false);
  }

  public default Trigger armGoToGroundPickUpNode() {
    return new Trigger(() -> false);
  }

  public default Trigger setArmByJoystick() {
    return new Trigger(() -> false);
  }

  public default Trigger setWristAlternatePose() {
    return new Trigger(() -> false);
  }

  public default Trigger openGrabber() {
    return new Trigger(() -> false);
  }
}
