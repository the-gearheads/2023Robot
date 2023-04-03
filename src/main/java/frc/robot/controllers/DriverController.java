package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverController {
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

  public default Trigger getPPLoadDebugForwardPath() {
    return new Trigger(() -> false);
  }

  public default Trigger getPPLoadDebugBackwardPath() {
    return new Trigger(() -> false);
  }

  public default Trigger getPPLoadDebugLeftPath() {
    return new Trigger(() -> false);
  }

  public default Trigger getPPLoadDebugRightPath() {
    return new Trigger(() -> false);
  }

  public default Trigger getPPGotoTag8() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger HIGH_SPEED() {
    return new Trigger(() -> false);
  }

  public default Trigger LOW_SPEED() {
    return new Trigger(() -> false);
  }

  public default Trigger backUpFromFeeder() {
    return new Trigger(() -> false);
  }

  /* Button to set wheels into X formation */
  public default Trigger getSetWheelXButton() {
    return new Trigger(() -> false);
  }

  public default double getSetHeadingPOV() { return -1; }

  public default double getPOV() {
    return -1;
  }

  public default Trigger testDockPath() {
    return new Trigger(() -> false);
  }

  public default Trigger alignToFeederStation() {
    return new Trigger(() -> false);
  }

  public default Trigger alignToGrid() {
    return new Trigger(() -> false);
  }

  public default Trigger getAutoLeft() {
    return new Trigger(() -> false);
  };

  public default Trigger getAutoCenter() {
    return new Trigger(() -> false);
  };

  public default Trigger getAutoRight() {
    return new Trigger(() -> false);
  };

  public default Trigger getAutoAlign() {
    return new Trigger(() -> false);
  }

  public default Trigger resetYaw(){
    return new Trigger(()->false);  
  }
}
