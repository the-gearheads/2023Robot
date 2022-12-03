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
  public CIMSteer(int id, Rotation2d angleOffset) {
    this.id=id;
    this.angleOffset = angleOffset;
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
    // motor.set(ControlMode.Position, angleToNative(angle));
  }
  public void setAngleMod360(double naiveDesiredAngle) {//So angle is [-pi,pi]. If the current angle is at -pi+0.01, and our desired angle pi-0.01, we don't want to cycle back to 2pi-0.02 degrees. 
  //If desired angle is 90 and our position is 450, we don't want to cycle 2pi times (occurs at the beginning only)
  double currentAngleMod360=getAngle()%360;
  double desiredAngleMod360=naiveDesiredAngle%360;
  double deltaBeta=desiredAngleMod360-currentAngleMod360;
  double delta=naiveDesiredAngle-getAngle();
  double deltaMod360=delta%360;
  if(Math.abs(deltaMod360) > Math.abs(deltaMod360-360)){
    deltaMod360=deltaMod360-360;
  }
  double desiredAngle=getAngle()+deltaMod360;
  motor.set(ControlMode.Position, angleToNative(desiredAngle+angleOffset.getDegrees()));
  if(id==34){
    SmartDashboard.putNumber("naiveDesiredAngle", naiveDesiredAngle);
    SmartDashboard.putNumber("currentAngel", getAngle());
    SmartDashboard.putNumber("currentAngleMod360", currentAngleMod360);
    SmartDashboard.putNumber("desiredAngleMod360", desiredAngleMod360);
    SmartDashboard.putNumber("delta", delta);
    SmartDashboard.putNumber("deltaMod360", deltaMod360);
    SmartDashboard.putNumber("final desiredAngle", desiredAngle);
  }
  }
  // public double getClosestMemberofDesiredAngleCongruenceFamilyMod360ToCurrentAngle(double currentAngle, double desiredAngle){
  //   double delta=desiredAngle-currentAngle;
  //   double member=currentAngle;
  //   while(Math.abs(delta)>180){
  //     if(delta>0){
  //       member+=360;
  //     }else{
  //       member+=360;
  //     }
  //   }
  // }
  public double mod360(double angle){//java is dumb and doesn't know how mod works
    return (angle%360)<0?angle%360+360:angle%360;
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

  public void setAngleOffset(Rotation2d angleOffset){
    //motor.setSelectedSensorPosition(motor.getSelectedSensorPosition() + angleToNative(angleOffset.minus(this.angleOffset).getDegrees()));
  }
}
