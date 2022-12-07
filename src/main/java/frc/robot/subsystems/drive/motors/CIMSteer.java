package frc.robot.subsystems.drive.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.MathUtils;

/* Uses Talon SRX to drive a CIM. Expects an absolute encoder connected. */
public class CIMSteer implements SteerMotor {
  WPI_TalonSRX motor;
  private Rotation2d angleOffset;
  private int id;
  private String ntWheelRootPath;
  public CIMSteer(int id, Rotation2d angleOffset, String ntWheelRootPath) {
    this.id=id;
    this.angleOffset = angleOffset;
    this.ntWheelRootPath = ntWheelRootPath;
    motor = new WPI_TalonSRX(id);
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0);
    motor.configFeedbackNotContinuous(false, 0);
    setPIDConstants(Constants.Drivetrain.STEER_F, Constants.Drivetrain.STEER_P, Constants.Drivetrain.STEER_I, Constants.Drivetrain.STEER_D);
    setBrakeMode(true);
  }

  private double angleToNative(double angle) {
    /* We need to take in angles in [-360, 360], and map that between -1024 and 1024. */
    double nativeAngle = angle/360;
    return nativeAngle * Constants.Drivetrain.ANALOG_UPR;
  }

  private double nativeToAngle(double nativeUnits) {
    double rotationCount = nativeUnits / Constants.Drivetrain.ANALOG_UPR;
    return rotationCount * 360;
  }

  public double getAngle() {
      // return nativeToAngle(getRawPosition());
    return nativeToAngle(getRawPosition()) - this.angleOffset.getDegrees();
  }

  public double getVelocity() {
    return nativeToAngle(getRawVelocity());
  }

  public void setAngle(double angle) {
    motor.set(ControlMode.Position, angleToNative(angle+angleOffset.getDegrees()));
  }

  public void setAngleMod360(double naiveDesiredAngle) {
    // Calculate the change in angle from the current angle to the desired angle
    double delta = naiveDesiredAngle - getAngle();

    // Ensure that the change in angle is within the range of [-180, 180] degrees
    double deltaMod360 = delta % 360;
    deltaMod360 = deltaMod360 < 360 ? deltaMod360 + 360 : deltaMod360;
    deltaMod360 = deltaMod360 > 180 ? deltaMod360 - 360 : deltaMod360;

    // Debugging output to SmartDashboard
    SmartDashboard.putNumber(ntWheelRootPath + "/Delta", delta);
    SmartDashboard.putNumber(ntWheelRootPath + "/Delta % 360", deltaMod360);

    // Calculate the final desired angle by adding the change in angle to the current angle
    double desiredAngle = getAngle() + deltaMod360;

    // Set the angle to the desired angle
    setAngle(desiredAngle);
  }

  public void setBrakeMode(boolean isBraking) {
    if(isBraking) {
      motor.setNeutralMode(NeutralMode.Brake);
    } else {
      motor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public double getRawPosition() {
    return motor.getSelectedSensorPosition();
  }

  private double getRawVelocity() {
    return motor.getSelectedSensorVelocity();
  }

  public void setPIDConstants(double kF, double kP, double kI, double kD){
    motor.config_kF(0, kF);
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
  }

}
