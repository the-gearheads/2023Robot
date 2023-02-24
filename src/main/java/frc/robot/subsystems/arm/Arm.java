// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
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
    VEL, POS;
  }

  private ArmControlMode controlMode;
  private double lastPos;
  private double lastNakedEncoderOutput;
  private double iHateThis;

  public Arm() {
    controlMode = ArmControlMode.VEL;
    configure();

    SmartDashboard.putData("arm/pPid", pid);
    SmartDashboard.putData("arm/vPid", velPid);
    SmartDashboard.putData("arm/ff", ff);
    setPoseGoal(getPosition());
    pid.reset(getPosition(), getVelocity());
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

  // [Possibly a] HACK: Manually wrap absolute encoder position if dP > 2 as there are no apis to do this for us
  public double getPosition() {
    var nakedEncoderOutput = encoder.getPosition();
    // if (nakedEncoderOutput - lastNakedEncoderOutput > 2) {
    //   iHateThis -= Math.PI * 2;
    // } else if (nakedEncoderOutput - lastNakedEncoderOutput < -2) {
    //   iHateThis += Math.PI * 2;
    // }
    // lastNakedEncoderOutput = nakedEncoderOutput;
    // return nakedEncoderOutput - ARM.ANGLE_OFFSET + iHateThis;
    return nakedEncoderOutput + ARM.ANGLE_OFFSET;
    // return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  private void configure() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    encoder.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    encoder.setPositionConversionFactor(2 * Math.PI);
    encoder.setVelocityConversionFactor(2 * Math.PI);

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

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    var pose = getPosition();
    var vel = getVelocity();

    SmartDashboard.putNumber("arm/pose", Units.radiansToDegrees(pose));
    SmartDashboard.putNumber("arm/vel", vel);
    SmartDashboard.putNumber("arm/posegoal", poseGoal);
    SmartDashboard.putNumber("arm/velgoal", velGoal);


    if(controlMode == ArmControlMode.VEL && MathUtil.applyDeadband(velGoal, 0.1) != 0) {
      // vel control
      velControl();
      SmartDashboard.putBoolean("arm/mode==POS", false);
    } else {
      // pos control
      posControl();
      SmartDashboard.putBoolean("arm/mode==POS", true);
    }
  }

  public void posControl(){
    var pose = getPosition();
    double pidval = pid.calculate(pose, poseGoal);
    double ffval = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity); // ff wants 0 parallel to floor in pos x
    setVoltage(ffval + pidval);

    
    SmartDashboard.putNumber("arm/ffval", ffval);
    SmartDashboard.putNumber("arm/pidval", pidval);
    SmartDashboard.putNumber("arm/setpoint", pid.getSetpoint().position);
  }

  public void velControl(){
    var pose = getPosition();
    var vel = getVelocity();

    /* Don't let ourselves go outside [-180, 0]  */
    if (((getPosition() > Units.degreesToRadians(20)) && velGoal < 0)
    ||  ((getPosition() < -Units.degreesToRadians(200)) && velGoal < 0)) {
      setVoltage(0);
      return;
    }

    if((getPosition() + velGoal*0.02 > Units.degreesToRadians(20))){
      velGoal = (0 - getPosition())/0.02;
      velGoal=0;
    }else if((getPosition() + velGoal*0.02 < -Units.degreesToRadians(200))){
      velGoal = (-Math.PI - getPosition())/0.02;
      velGoal=0;
    }

    double ffval = ff.calculate(pose, velGoal);
    double pidval = velPid.calculate(vel, velGoal);
    
    setVoltage(ffval + pidval);
    setPoseGoal(pose);
    pid.reset(getPosition(), getVelocity());
  }
}
