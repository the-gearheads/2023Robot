package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerInterface {

  /** Axis used for moving forwards and backwards. Value between -1 and 1 */
  public default double getXMoveAxis() {
    return 0.0;
  }

  /** Axis used for moving left and right. Value between -1 and 1 */
  public default double getYMoveAxis() {
    return 0.0;
  }

  /** Axis used for rotating left and right. Value between -1 and 1. */
  public default double getRotateAxis() {
    return 0.0;
  }

  public default double getSwerveDebugAngleX() {
    return 0.0;
  }

  public default double getSwerveDebugAngleY() {
    return 0.0;
  }

  public default double getSwerveDebugSpeedAxis() {
    return 0.0;
  }

  public default Trigger getSwerveDebugEnableButton() {
    return new Trigger();
  }

  public default Trigger getSwerveDebugOptimizeEnabledButton() {
    return new Trigger();
  }

}
