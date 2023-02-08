package frc.robot.controllers;

public interface OperatorController {
  public default double getArmAxis() {
    return 0.0;
  }
}
