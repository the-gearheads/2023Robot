package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class SingleXboxController implements ControllerInterface {

  XboxController controller;

  public SingleXboxController(int port) {
    controller = new XboxController(port);
  }

  private double deadband(double num) {
    return MathUtil.applyDeadband(num, Constants.Controllers.JOYSTICK_DEADBAND);
  }

  public double getXMoveAxis() {
    return deadband(controller.getLeftY());
  }

  public double getYMoveAxis() {
    return deadband(controller.getLeftX());
  }

  public double getRotateAxis() {
    return deadband(controller.getRightX());
  }

  public double getSwerveDebugAngleX() {
    return 0.0;
  }

  public double getSwerveDebugAngleY() {
    return 0.0;
  }

  public double getSwerveDebugSpeedAxis() {
    return 0.0;
  }

  public Trigger getSwerveDebugEnableButton() {
    return new Trigger(()-> controller.getYButton()).debounce(0.01);
  }

  public Trigger getSwerveDebugOptimizeEnabledButton() {
    return new Trigger(() -> controller.getXButton()).debounce(0.01);
  }


}
