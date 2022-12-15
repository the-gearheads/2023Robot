package frc.robot.subsystems.drive.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;

/* Uses Talon SRX to drive a CIM. Expects an absolute encoder connected. */
public class CIMSteer implements SteerMotor {
  WPI_TalonSRX motor;
  TalonSRXSimCollection sim;
  private Rotation2d angleOffset;
  
  public CIMSteer(int id, Rotation2d angleOffset) {
    this.angleOffset = angleOffset;
    motor = new WPI_TalonSRX(id);
    sim = motor.getSimCollection();
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0);
    motor.configFeedbackNotContinuous(false, 0);
    motor.configOpenloopRamp(Drivetrain.STEER_RAMP_RATE);
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
    if(SmartDashboard.getBoolean("/Swerve/CoolWheelStuff", true)) {
      angle = angleMod360(angle);
    }

    motor.set(ControlMode.Position, angleToNative(angle+angleOffset.getDegrees()));
  }

  public double angleMod360(double naiveDesiredAngle) {
    // Calculate the change in angle from the current angle to the desired angle
    double delta = naiveDesiredAngle - getAngle();

    // Ensure that the change in angle is within the range of [-180, 180] degrees
    double deltaMod360 = delta % 360;
    deltaMod360 = deltaMod360 < 360 ? deltaMod360 + 360 : deltaMod360;
    deltaMod360 = deltaMod360 > 180 ? deltaMod360 - 360 : deltaMod360;

    // Calculate the final desired angle by adding the change in angle to the current angle
    double desiredAngle = getAngle() + deltaMod360;

    return desiredAngle;
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
    motor.setSelectedSensorPosition(motor.getSelectedSensorPosition() + angleToNative(angleOffset.minus(this.angleOffset).getDegrees()));
  }

  public void setSimPosition(double angle) {
    sim.setAnalogPosition((int)angleToNative(angle));
  }

  public void setSimVelocity(double vel) {
    sim.setAnalogVelocity((int)angleToNative(vel));
  }

  public double getAppliedVolts() {
    return motor.getMotorOutputPercent();
  }
  
  public FlywheelSim getSim() {
    return new FlywheelSim(LinearSystemId.identifyVelocitySystem(Drivetrain.Sim.CIMSteer.kV, Drivetrain.Sim.CIMSteer.kA), Drivetrain.Sim.CIMSteer.motor, Constants.Drivetrain.STEER_GEAR_RATIO);
  }

  public SlotConfiguration getSlotConfig() {
    var config = new SlotConfiguration();
    motor.getSlotConfigs(config);
    return config;
  }

  /* getSlotConfig() is way too slow so we make the assumption that the value is what we last set it to
   * I am very good at writing cursed code. 
   */
  double lastP = Drivetrain.STEER_P;
  double lastI = Drivetrain.STEER_I;
  double lastD = Drivetrain.STEER_D;
  double lastF = 0;

  public void initSendable(SendableBuilder b) {
    b.addDoubleProperty("steer/currentAngle", this::getAngle, null);
    b.addDoubleProperty("steer/AppliedVolts", this::getAppliedVolts, null);
    b.addDoubleProperty("steer/Velocity", this::getVelocity, null);

    b.addDoubleProperty("steer/pid/P", ()->{return lastP;}, (double P)->{lastP = P; motor.config_kP(0, P);});
    b.addDoubleProperty("steer/pid/I", ()->{return lastI;}, (double I)->{lastI = I; motor.config_kI(0, I);});
    b.addDoubleProperty("steer/pid/D", ()->{return lastD;}, (double D)->{lastD = D; motor.config_kD(0, D);});
    b.addDoubleProperty("steer/pid/F", ()->{return lastF;}, (double F)->{lastF = F; motor.config_kF(0, F);});

  }

}
