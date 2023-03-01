package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Controllers {

  private Controllers() {}

  private static String[] lastControllerNames = new String[6];

  public static DriverController driverController;
  public static OperatorController operatorController;

  /** Returns true if the connected controllers have changed since last called. */
  public static boolean didControllersChange() {
    boolean hasChanged = false;

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      String name = DriverStation.getJoystickName(i);
      if (!name.equals(lastControllerNames[i])) {
        hasChanged = true;
        lastControllerNames[i] = name;
      }
    }

    return hasChanged;
  }

  public static void updateActiveControllerInstance() {
    // Defaults, since a NullPointerException would be far worse than any warnings
    driverController = new DriverController() {};
    operatorController = new OperatorController() {};

    boolean foundOperatorController = false;
    boolean foundDriveController = false;

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      String joyName = DriverStation.getJoystickName(i);
      if (joyName.equals(""))
        continue;
      if (!foundOperatorController
          && (joyName.toLowerCase().contains("logitech") || joyName.toLowerCase().contains("keyboard"))) {
        System.out.println("Found logitech operator controller on port " + i);
        operatorController = new LogitechOperatorController(i);
        foundOperatorController = true;
        continue;
      } else if (!foundOperatorController
          && (joyName.toLowerCase().contains("16000m"))) {
        System.out.println("Found Thrustmaster operator controller on port " + i);
        operatorController = new ThrustMaster(i);
        foundOperatorController = true;
        continue;
      }
      if (!foundOperatorController
          && (joyName.toLowerCase().contains("16000m"))) {
        System.out.println("Found Thrustmaster operator controller on port " + i);
        operatorController = new ThrustMaster(i);
        foundOperatorController = true;
        continue;
      }
      // Fallback
      if (!foundDriveController) {
        System.out.println("Found xbox drive controller controller on port " + i);
        driverController = new XboxDriverController(i);
        foundDriveController = true;
      }
    }
  }

  public static double deadband(double num) {
    return MathUtil.applyDeadband(num, Constants.CONTROLLERS.JOYSTICK_DEADBAND);
  }
}
