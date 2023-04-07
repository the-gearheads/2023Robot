package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorController {
  public default double getArmAxis() {
    return 0.0;
  }

  public default double getPOVAngle() {
    return -1;
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

  public default Trigger frontPickup() {
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

  public default Trigger throwCube() {
    return new Trigger(() -> false);
  }

  public default Trigger signalCube() {
    return new Trigger(() -> false);
  }

  public default Trigger signalCone() {
    return new Trigger(() -> false);
  }

  public default Trigger reconfigEVERYTHING() {
    return new Trigger(() -> false);
  }

  public default Trigger autoGrab() {
    return new Trigger(() -> false);
  }
}