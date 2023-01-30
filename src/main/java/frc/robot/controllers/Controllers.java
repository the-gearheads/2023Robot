package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;

public class Controllers {

  private Controllers() {}

  private static String[] lastControllerNames = new String[6];

  public static ControllerInterface activeController;

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
    /* Just find the first Xbox controller */
    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      if (!DriverStation.getJoystickName(i).equals("")) {
        activeController = new SingleXboxController(i);
        return;
      }
    }
    // No controller found, but a NullPointerException would be far worse than any warnings
    activeController = new SingleXboxController(0);
  }
}
