package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Controllers {

  private Controllers() {}

  private static String[] lastControllerNames = new String[6];

  public static DriverController driverController;
  public static OperatorController operatorController;
  public static AlignController alignController;

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
    alignController = new AlignController() {};

    boolean foundOperatorController = false;
    boolean foundDriveController = false;
    boolean foundAlignController = false;

    for (int i = 0; i < DriverStation.kJoystickPorts; i++) {
      String joyName = DriverStation.getJoystickName(i);
      if (joyName.equals(""))
        continue;
      if (!foundOperatorController) {
        foundOperatorController = true;
        if (joyName.toLowerCase().contains("logitech")) {
          operatorController = new LogitechOperatorController(i);
          Logger.getInstance().recordOutput("controllers/operator controller",
              "Found logitech operator controller on port " + i);
        } else if (joyName.toLowerCase().contains("16000m")) {
          Logger.getInstance().recordOutput("controllers/operator controller",
              "Found thrustmaster operator controller on port " + i);
          operatorController = new ThrustMaster(i);
        } else if (joyName.toLowerCase().contains("USB Gamepad")) {
          Logger.getInstance().recordOutput("controllers/operator controller",
              "Found Sega operator controller on port " + i);
          operatorController = new SegaOperatorController(i);
        } else if (joyName.toLowerCase().contains("keyboard")) {
          operatorController = new LogitechOperatorController(i);
          Logger.getInstance().recordOutput("controllers/operator controller",
              "Found keyboard operator controller on port " + i);
        } else {
          foundOperatorController = false;
        }
        if (foundOperatorController)
          continue;
      }

      if (!foundDriveController) {
        foundDriveController = true;
        if (joyName.toLowerCase().contains("xbox")) {
          Logger.getInstance().recordOutput("controllers/driver controller",
              "Found xbox drive controller controller on port " + i);
          driverController = new XboxDriverController(i);
        } else if (joyName.toLowerCase().contains("keyboard")) {
          Logger.getInstance().recordOutput("controllers/driver controller",
              "Found xbox drive controller controller on port " + i);
          driverController = new XboxDriverController(i);
        } else if (joyName.toLowerCase().contains("gamesir")) {
          Logger.getInstance().recordOutput("controllers/driver controller",
              "Found GAMESIR DRIVE controller controller on port " + i);
          driverController = new XboxDriverController(i);
        }else {
          foundDriveController = false;
        }
        if (foundDriveController)
          continue;
      }

      if (!foundAlignController) {
        foundAlignController = true;
        if (joyName.toLowerCase().contains("keyboard")) {
          Logger.getInstance().recordOutput("controllers/align controller",
              "Found keyboard align controller controller on port " + i);
          alignController = new KeypadAlignController(i);
        } else {
          foundAlignController = false;
        }
        if (foundAlignController)
          continue;
      }
    }
  }

  public static double deadband(double num) {
    return MathUtil.applyDeadband(num, Constants.CONTROLLERS.JOYSTICK_DEADBAND);
  }
}
