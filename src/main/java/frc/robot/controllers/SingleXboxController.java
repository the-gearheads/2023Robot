package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class SingleXboxController implements ControllerInterface {

    XboxController controller;

    public SingleXboxController(int port) {
        controller = new XboxController(port);
    }

    public double getXMoveAxis() {
        return MathUtil.applyDeadband(controller.getLeftY(), Constants.Controllers.JOYSTICK_DEADBAND);
    }

    public double getYMoveAxis() {
        return MathUtil.applyDeadband(controller.getLeftX(), Constants.Controllers.JOYSTICK_DEADBAND);
    }

    public double getRotateAxis() {
        return MathUtil.applyDeadband(controller.getRightX(), Constants.Controllers.JOYSTICK_DEADBAND);
    }

}
