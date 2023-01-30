package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
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

}
