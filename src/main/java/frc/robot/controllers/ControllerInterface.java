package frc.robot.controllers;

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

}
