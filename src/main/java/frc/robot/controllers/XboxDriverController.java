package frc.robot.controllers;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

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
    if (Constants.getMode() == Constants.RobotMode.SIM) {
      return -Controllers.deadband(controller.getRawAxis(2));
    }
    return -Controllers.deadband(controller.getRightX());
  }

  public Trigger getPPLoadDebugForwardPath() {
    return new JoystickButton(controller, XboxController.Button.kY.value);
  }

  public Trigger getPPLoadDebugBackwardPath() {
    return new JoystickButton(controller, XboxController.Button.kA.value);
  }

  public Trigger getPPLoadDebugLeftPath() {
    return new JoystickButton(controller, XboxController.Button.kX.value);
  }

  public Trigger getPPLoadDebugRightPath() {
    return new JoystickButton(controller, XboxController.Button.kB.value);
  }

  public Trigger getPPGotoTag8() {
    return new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  }

  public Trigger HIGH_SPEED() {
    return new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  }
  public Trigger LOW_SPEED() {
    return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  }

  public Trigger getResetPoseButton() {
    return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  }

  /* Probably should change to something better but all the good buttons are used by the debug stuff */
  public Trigger getSetWheelXButton() {
    return new JoystickButton(controller, XboxController.Button.kStart.value);
  }
}
