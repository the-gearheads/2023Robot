// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM;
import frc.robot.util.RevConfigUtils;
import frc.robot.util.SendableArmFeedforward;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  public boolean configureHasRan = false;
  private int zeroCount = 0;
  private TrapezoidProfile.State poseGoal = new TrapezoidProfile.State();
  private double velGoal = 0;
  protected CANSparkMax motor = new CANSparkMax(ARM.ARM_ID, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  protected ProfiledPIDController pid =
      new ProfiledPIDController(ARM.ARM_POS_PID[0], ARM.ARM_POS_PID[1], ARM.ARM_POS_PID[2], ARM.ARM_CONSTRAINTS);
  protected ProfiledPIDController velPid =
      new ProfiledPIDController(ARM.ARM_VEL_PID[0], ARM.ARM_VEL_PID[1], ARM.ARM_VEL_PID[2], ARM.ARM_VEL_CONSTRAINTS);
  protected SendableArmFeedforward ff =
      new SendableArmFeedforward(ARM.ARM_FF[0], ARM.ARM_FF[1], ARM.ARM_FF[2], ARM.ARM_FF[3]);
  private double lastTime = Timer.getFPGATimestamp();
  private double lastVel = 0;
  private double acceleration;

  public enum ArmControlMode {
    VEL("Velocity"), POS("Position"), VOLTAGE("voltage");

    public final String name;

    private ArmControlMode(String name) {
      this.name = name;
    }
  }

  private ArmControlMode controlMode;
  private boolean prevDisabled;

  public Arm() {
    controlMode = ArmControlMode.VEL;
    RevConfigUtils.configure(this::configure, "Arm");

    SmartDashboard.putData("Arm/pPid", pid);
    SmartDashboard.putData("Arm/vPid", velPid);
    SmartDashboard.putData("Arm/ff", ff);
    setPoseGoal(getPose());
    resetPIDs();
    prevDisabled = !DriverStation.isEnabled();
  }

  public void setControlMode(ArmControlMode mode) {
    controlMode = mode;
  }

  public void resetPIDs() {
    pid.reset(getPose(), getVel());
    velPid.reset(getVel(), 0);
  }

  public TrapezoidProfile.State getPoseGoal() {
    return poseGoal;
  }

  public void setPoseGoal(double poseGoal) {
    this.setPoseGoal(new TrapezoidProfile.State(poseGoal, 0));
  }

  public void setPoseGoal(TrapezoidProfile.State poseGoal) {
    if (!inAllowableRange(poseGoal.position)) {
      DriverStation.reportWarning("Arm Pose Handler TRIGGERED!!!", true);
      Logger.getInstance().recordOutput("Arm Pose Handler TRIGGERED!!!", true);
      return;
    }
    Logger.getInstance().recordOutput("Arm Pose Handler TRIGGERED!!!", false);
    this.poseGoal = poseGoal;
  }

  public double getVelGoal() {
    return velGoal;
  }

  public void setVelGoal(double newGoal) {
    velGoal = newGoal;
  }

  public double getPose() {
    return encoder.getPosition() + ARM.ANGLE_OFFSET;
  }

  public double getVel() {
    return encoder.getVelocity();
  }

  public void setVoltage(double volts) {
    Logger.getInstance().recordOutput("Arm/Appliedvolts", volts);
    motor.setVoltage(volts);
  }

  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public void periodic() {
    if (prevDisabled && DriverStation.isEnabled()) {
      setPoseGoal(getPose());
      resetPIDs();
    }
    prevDisabled = !DriverStation.isEnabled();

    if (sensorErrorHandler()) {
      DriverStation.reportError("OUR ZERO ERROR IN ARM", true);
      setVoltage(0);
      if (configureHasRan == false) {
        RevConfigUtils.configure(this::configure, "Arm");
      }
      configureHasRan = true;
      return;
    }

    var pose = getPose();
    var vel = getVel();
    var volts = 0.0;

    /* Calculate accleration. IT'S DOUBLE DIFFERENTIATION TIME*/
    acceleration = (vel - lastVel) / (Timer.getFPGATimestamp() - lastTime);

    Logger.getInstance().recordOutput("Arm/CurrentPose", pose);
    Logger.getInstance().recordOutput("Arm/CurrentVel", vel);
    Logger.getInstance().recordOutput("Arm/Acceleration", acceleration);
    Logger.getInstance().recordOutput("Arm/Pose/Pose Goal", poseGoal.position);
    Logger.getInstance().recordOutput("Arm/Pose/Vel Goal", poseGoal.velocity);
    Logger.getInstance().recordOutput("Arm/Vel/Goal", velGoal);
    Logger.getInstance().recordOutput("Arm/ControlMode", controlMode.name);
    Logger.getInstance().recordOutput("Arm/Pose/pose setpoint", pid.getSetpoint().position);
    Logger.getInstance().recordOutput("Arm/Pose/vel setpoint", pid.getSetpoint().velocity);
    Logger.getInstance().recordOutput("Arm/current command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "");

    Logger.getInstance().recordOutput("Arm/Vel/Setpoint", velPid.getSetpoint().position);

    /* Controlled entirely via setVoltage() */
    if (controlMode == ArmControlMode.VOLTAGE)
      return;

    if (controlMode == ArmControlMode.VEL) {
      volts = velControl();
    } else {
      volts = poseControl();
    }

    lastVel = getVel();
    lastTime = Timer.getFPGATimestamp();

    volts = applySoftLimit(volts);
    setVoltage(volts);
  }

  public boolean sensorErrorHandler() {
    boolean hasFaults = motor.getFault(FaultID.kCANTX) || motor.getFault(FaultID.kCANRX);
    boolean hasStickyFaults = motor.getStickyFault(FaultID.kCANTX) || motor.getStickyFault(FaultID.kCANRX);
    var pose = encoder.getPosition();

    if (pose == 0 || pose > 2000 || pose < -2000) {
      zeroCount++;
    }

    var zeroCountFault = zeroCount > 1;
    Logger.getInstance().recordOutput("Arm/Faults/Zero Count Fault", zeroCountFault);
    Logger.getInstance().recordOutput("Arm/Faults/Fault", hasFaults);
    Logger.getInstance().recordOutput("Arm/Faults/Sticky Fault", hasStickyFaults);

    if (hasStickyFaults) {
      DriverStation.reportWarning("Arm Sticky Fault", true);
    }
    return zeroCountFault || hasFaults;
  }

  public double poseControl() {
    var pose = getPose();
    double pidval = pid.calculate(pose, poseGoal);
    double ffval = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity); // ff wants 0 parallel to floor in pos x

    Logger.getInstance().recordOutput("Arm/Pose/FFval", ffval);
    Logger.getInstance().recordOutput("Arm/Pose/PIDval", pidval);
    Logger.getInstance().recordOutput("Arm/Pose/Error", pid.getPositionError());

    velPid.reset(getVel(), 0); // do not replace 0 with acceleration
    return ffval + pidval;
  }

  public double velControl() {
    var pose = getPose();
    var vel = getVel();

    /* Prospective Soft Limit */
    if ((getPose() + velGoal * 0.02 > ARM.MAX_ANGLE)) {
      velGoal = 0;
    } else if ((getPose() + velGoal * 0.02 < ARM.MIN_ANGLE)) {
      velGoal = 0;
    }

    double ffval = ff.calculate(pose, velGoal);
    double pidval = velPid.calculate(vel, velGoal);

    Logger.getInstance().recordOutput("Arm/Vel/FFval", ffval);
    Logger.getInstance().recordOutput("Arm/Vel/PIDval", pidval);
    Logger.getInstance().recordOutput("Arm/Vel/Error", velPid.getPositionError());

    setPoseGoal(pose);
    pid.reset(getPose(), getVel());
    return ffval + pidval;
  }

  private double applySoftLimit(double volts) {
    if (((getPose() > ARM.MAX_ANGLE) && volts > 0) || ((getPose() < ARM.MIN_ANGLE) && volts < 0)) {
      return 0;
    }
    return volts;
  }

  public boolean inAllowableRange(double pose) {
    return pose <= ARM.MAX_ANGLE && pose >= ARM.MIN_ANGLE;
  }

  private ArrayList<REVLibError> configure() {
    ArrayList<REVLibError> e = new ArrayList<>();
    e.add(motor.restoreFactoryDefaults());
    motor.setInverted(false);
    e.add(motor.setSmartCurrentLimit(50));
    e.add(encoder.setInverted(false));
    e.add(motor.setIdleMode(IdleMode.kBrake));
    e.add(encoder.setPositionConversionFactor(360));
    e.add(encoder.setVelocityConversionFactor(360));

    //currently useless
    e.add(motor.setSoftLimit(SoftLimitDirection.kForward, (float) (ARM.MAX_ANGLE - ARM.ANGLE_OFFSET)));
    e.add(motor.setSoftLimit(SoftLimitDirection.kReverse, (float) (ARM.MIN_ANGLE - ARM.ANGLE_OFFSET)));
    e.add(motor.enableSoftLimit(SoftLimitDirection.kForward, false));
    e.add(motor.enableSoftLimit(SoftLimitDirection.kReverse, false));

    /* Status 0 governs applied output, faults, and whether is a follower. Not important for this. */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20));
    /* Integrated motor position isn't important here. */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500));
    /* Don't have an analog sensor */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    /* Don't have an alternate encoder */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    /* Have a duty cycle encoder */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20));
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20));

    return e;
  }
}
