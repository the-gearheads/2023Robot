package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class SingleXboxController implements ControllerInterface {

  public XboxController controller;

  public SingleXboxController(int port) {
    controller = new XboxController(port);
  }

  private double deadband(double num) {
    return MathUtil.applyDeadband(num, Constants.Controllers.JOYSTICK_DEADBAND);
  }

  @Override
  public double getXMoveAxis() {
    return -deadband(controller.getLeftY());
  }

  @Override
  public double getYMoveAxis() {
    return -deadband(controller.getLeftX());
  }

  @Override
  public double getRotateAxis() {
    return -deadband(controller.getRightX());
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

  public Trigger getResetPoseButton() {
    return new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  }

}
