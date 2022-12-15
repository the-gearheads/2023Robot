package frc.robot.subsystems.drive.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Drivetrain;

public class NEODrive implements DriveMotor {
  /**
    * Very condensed version of this:
    * First. we use the gear ratio in reverse by multiplying by 1/DRIVE_GEAR_RATIO. This gives us the wheel rotation count.
    * Then, we multiple by the circumference to get distrance traveled (in meters).
    */
  public double conversionFactor = Drivetrain.WHEEL_CIRCUMFERENCE / Drivetrain.DRIVE_GEAR_RATIO;;
  CANSparkMax motor;
  RelativeEncoder encoder;
  double simPos, simVel;
  PIDController pid = new PIDController(Drivetrain.DRIVE_P, Drivetrain.DRIVE_I, Drivetrain.DRIVE_D);
  SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Drivetrain.DRIVE_KS, Drivetrain.DRIVE_KV);

  public NEODrive(int id, boolean invertSteer) {
    /* SparkMAX is honestly really nice and simple */
    motor = new CANSparkMax(id, MotorType.kBrushless);
    if(!Robot.isReal()) {
      REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
    }
    encoder = motor.getEncoder();

    encoder.setPositionConversionFactor(conversionFactor);
    encoder.setVelocityConversionFactor(conversionFactor * (1/60.0));
    motor.setInverted(invertSteer);

    setBrakeMode(true);

    pid.setTolerance(0.05, 0.05);
    motor.setOpenLoopRampRate(Drivetrain.DRIVE_RAMP_RATE);
  }

  public void setBrakeMode(boolean isBraking) {
    if (isBraking) {
      motor.setIdleMode(IdleMode.kBrake);
    } else {
      motor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getAppliedVolts() {
    return motor.getAppliedOutput();
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  public void tickPID() {
    setVoltage(ff.calculate(pid.getSetpoint()) + pid.calculate(getVelocity()));
  }

  public void setSpeed(double speed) {
    pid.setSetpoint(speed);
  }

  public void zeroEncoders() {
    encoder.setPosition(0);
    simPos = 0;
    simVel = 0;
  }

  public double getPosition() {
    return Robot.isReal() ? encoder.getPosition() : simPos;
  }

  public double getVelocity() {
    return Robot.isReal() ? encoder.getVelocity() : simVel;
  }

  public void simSetPosition(double pos) {
    simPos = pos;
  }

  public void simSetVelocity(double vel) {
    simVel = vel;
  }

  public FlywheelSim getSim() {
    return new FlywheelSim(LinearSystemId.identifyVelocitySystem(Drivetrain.Sim.NEODrive.kV, Drivetrain.Sim.NEODrive.kA), Drivetrain.Sim.NEODrive.motor, Constants.Drivetrain.DRIVE_GEAR_RATIO);
  }

  public void initSendable(SendableBuilder b) {
    b.addDoubleProperty("drive/vel", this::getVelocity, null);
    b.addDoubleProperty("drive/pos", this::getPosition, null);
    b.addDoubleProperty("drive/appliedVolts", this::getAppliedVolts, null);
    b.addDoubleProperty("drive/pid/P", pid::getP, pid::setP);
    b.addDoubleProperty("drive/pid/I", pid::getI, pid::setI);
    b.addDoubleProperty("drive/pid/D", pid::getD, pid::setD);
    b.addDoubleProperty("drive/ff/kS", ()->{ return ff.ks; }, (double kS) -> {ff = new SimpleMotorFeedforward(kS, ff.kv);});
    b.addDoubleProperty("drive/ff/kV", ()->{ return ff.kv; }, (double kV) -> {ff = new SimpleMotorFeedforward(ff.ks, kV);});
  }
}
