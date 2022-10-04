package frc.robot.subsystems.drive.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.util.MathUtils;

/* Uses Talon SRX to drive a CIM. Expects an absolute encoder connected. */
public class CIMSteer implements SteerMotor {
  WPI_TalonSRX motor;
  public CIMSteer(int id) {
    motor = new WPI_TalonSRX(id);
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0);
    motor.configFeedbackNotContinuous(false, 0);
    motor.config_kF(0, Constants.Drivetrain.STEER_F);
    motor.config_kP(0, Constants.Drivetrain.STEER_P);
    motor.config_kI(0, Constants.Drivetrain.STEER_I);
    motor.config_kD(0, Constants.Drivetrain.STEER_D);
    setBrakeMode(true);
  }

  private double angleToNative(double angle) {
    /* We need to take in angles in [-360, 360], and map that between -1024 and 1024. */
    double nativeAngle = MathUtils.scale(-360, 360, -Constants.Drivetrain.ANALOG_UPR, Constants.Drivetrain.ANALOG_UPR, angle);
    return nativeAngle;
  }

  /* I -think- this is correct? */
  private double nativeToAngle(double nativeUnits) {
    double rotationCount = nativeUnits / Constants.Drivetrain.ANALOG_UPR;
    return rotationCount * 360;
  }

  public double getAngle() {
    return nativeToAngle(getRawPosition());
  }

  public double getVelocity() {
    return nativeToAngle(getRawVelocity());
  }

  public void setAngle(double angle) {
    motor.set(ControlMode.Position, angleToNative(angle));
  }

  public void setBrakeMode(boolean isBraking) {
    if(isBraking) {
      motor.setNeutralMode(NeutralMode.Brake);
    } else {
      motor.setNeutralMode(NeutralMode.Coast);
    }
  }

  private double getRawPosition() {
    return motor.getSelectedSensorPosition();
  }

  private double getRawVelocity() {
    return motor.getSelectedSensorVelocity();
  }
}
