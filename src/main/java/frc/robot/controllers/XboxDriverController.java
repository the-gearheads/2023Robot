package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxDriverController implements DriverController {

  public XboxController controller;

  public XboxDriverController(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getXMoveAxis() {
    Logger.getInstance().recordOutput("TeleopDrive/Raw/xSpd", -controller.getLeftY());
    return -Controllers.deadband(controller.getLeftY());
  }

  public Trigger backUpFromFeeder() {
    return new JoystickButton(controller, XboxController.Button.kA.value);
  }

  @Override
  public double getYMoveAxis() {
    return -Controllers.deadband(controller.getLeftX());
  }


  @Override
  public double getRotateAxis() {
    if (cardinalIsSet().getAsBoolean()) {
      return 0;
    }

    return -Controllers.deadband(controller.getRightX());
  }

  public Trigger alignToFeederStation() {
    return new JoystickButton(controller, XboxController.Button.kB.value);
  }

  public Trigger alignToGrid() {
    return new JoystickButton(controller, XboxController.Button.kY.value);
  }

  public Trigger HIGH_SPEED() {
    return new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  }

  public Trigger LOW_SPEED() {
    return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  }

  // private Trigger getRotateButton() {
  //   return new Trigger(() -> {
  //     return controller.getRightTriggerAxis() > 0.7;
  //   });
  // }

  public Trigger getFastPickUpBtn() {
    return new Trigger(() -> {
      return controller.getRightTriggerAxis() > 0.7;
    });
  }

  public Trigger getResetPoseButton() {
    return new JoystickButton(controller, XboxController.Button.kX.value);
  }

  /* Probably should change to something better but all the good buttons are used by the debug stuff */
  public Trigger getSetWheelXButton() {
    return new JoystickButton(controller, XboxController.Button.kStart.value);
  }

  public double getSetHeadingPOV() {
    if (controller.getRightY() > 0.9)
      return 180;
    // if (controller.getRightY() < -0.9)
    //   return 0;
    return -1;
  }

  public Trigger cardinalIsSet() {
    return new Trigger(() -> {
      return getSetHeadingPOV() != -1;
    });
  }

  public double getPOV() {
    return controller.getPOV();
  };

  public Trigger testDockPath() {
    return new JoystickButton(controller, XboxController.Button.kY.value);
  }

  public Trigger getAutoAlign() {

    return new Trigger(() -> {
      var left = getAutoLeft().getAsBoolean();
      var center = getAutoCenter().getAsBoolean();
      var right = getAutoRight().getAsBoolean();
      return (left || center || right);
    });
  }

  public Trigger getAutoLeft() {
    return new JoystickButton(controller, XboxController.Button.kX.value);
  };

  public Trigger getAutoCenter() {
    return new JoystickButton(controller, XboxController.Button.kY.value);
  };

  public Trigger getAutoRight() {
    return new JoystickButton(controller, XboxController.Button.kB.value);
  };

  public Trigger resetYaw() {
    return new Trigger(() -> {
      var val = controller.getLeftTriggerAxis();
      return Math.abs(val) > 0.9;
    });
  }

  public Trigger autoGrab() {
    return new Trigger(() -> {
      return Math.abs(controller.getRightTriggerAxis()) > 0.9;
    });
  }
}
