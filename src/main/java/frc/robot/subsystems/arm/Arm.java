// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM;
import frc.robot.util.SendableArmFeedforward;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  private double poseGoal = 0;
  private double velGoal = 0;
  protected CANSparkMax motor = new CANSparkMax(ARM.ARM_ID, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private ProfiledPIDController pid =
      new ProfiledPIDController(ARM.ARM_POS_PID[0], ARM.ARM_POS_PID[1], ARM.ARM_POS_PID[2], ARM.ARM_CONSTRAINTS);
  private PIDController velPid = new PIDController(ARM.ARM_VEL_PID[0], ARM.ARM_VEL_PID[1], ARM.ARM_VEL_PID[2]);
  private SendableArmFeedforward ff = new SendableArmFeedforward(ARM.ARM_FF[0], ARM.ARM_FF[1], ARM.ARM_FF[2]);

  public enum ArmControlMode {
    VEL("Velocity"), POS("Position");

    public final String name;

    private ArmControlMode(String name) {
      this.name = name;
    }
  }

  private ArmControlMode controlMode;

  public Arm() {
    controlMode = ArmControlMode.VEL;
    configure();

    SmartDashboard.putData("arm/pPid", pid);
    SmartDashboard.putData("arm/vPid", velPid);
    SmartDashboard.putData("arm/ff", ff);
    setPoseGoal(getPose());
    pid.reset(getPose(), getVelocity());
  }

  public void setControlMode(ArmControlMode mode) {
    // pid.reset(getPosition(), getVelocity());
    velPid.reset();
    controlMode = mode;
  }

  public double getPoseGoal() {
    return poseGoal;
  }

  public void setPoseGoal(double newGoal) {
    poseGoal = newGoal;
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

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void setVoltage(double volts) {
    Logger.getInstance().recordOutput("Arm/Appliedvolts", volts);
    motor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    var pose = getPose();
    var vel = getVelocity();
    var volts=0.0;

    Logger.getInstance().recordOutput("Arm/CurrentPose", pose);
    Logger.getInstance().recordOutput("Arm/CurrentVel", vel);
    Logger.getInstance().recordOutput("Arm/Pose/Goal", poseGoal);
    Logger.getInstance().recordOutput("Arm/Vel/Goal", velGoal);
    Logger.getInstance().recordOutput("Arm/ControlMode", controlMode.name);
    Logger.getInstance().recordOutput("Arm/Pose/Setpoint", pid.getSetpoint().position);

    if (controlMode == ArmControlMode.VEL && MathUtil.applyDeadband(velGoal, 0.1) != 0) {
      volts=velControl();
    } else {
      volts=poseControl();
    }

    volts=applySoftLimit(volts);
    setVoltage(volts);
  }

  public double poseControl() {
    var pose = getPose();
    double pidval = pid.calculate(pose, poseGoal);
    double ffval = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity); // ff wants 0 parallel to floor in pos x

    Logger.getInstance().recordOutput("Arm/Pose/FFval", ffval);
    Logger.getInstance().recordOutput("Arm/Pose/PIDval", pidval);
    Logger.getInstance().recordOutput("Arm/Pose/Error", pid.getPositionError());

    return ffval + pidval;
  }

  public double velControl() {
    var pose = getPose();
    var vel = getVelocity();

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
    pid.reset(getPose(), getVelocity());
    return ffval + pidval;
  }

  private double applySoftLimit(double volts){
    if (((getPose() > ARM.MAX_ANGLE) && volts > 0) || ((getPose() < ARM.MIN_ANGLE) && volts < 0)) {
      return 0;
    }
    return volts;
  }

  private void configure() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    encoder.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    encoder.setPositionConversionFactor(360);
    encoder.setVelocityConversionFactor(360);

    //currently useless
    motor.setSoftLimit(SoftLimitDirection.kForward, (float) (ARM.MAX_ANGLE - ARM.ANGLE_OFFSET));
    motor.setSoftLimit(SoftLimitDirection.kReverse, (float) (ARM.MIN_ANGLE - ARM.ANGLE_OFFSET));
    motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    /* Status 0 governs applied output, faults, and whether is a follower. Not important for this. */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    /* Integrated motor position isn't important here. */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    /* Don't have an analog sensor */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* Have a duty cycle encoder */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }
}
