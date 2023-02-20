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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  private double goal = 0;
  protected CANSparkMax motor = new CANSparkMax(ARM.ARM_ID, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private ProfiledPIDController pid =
      new ProfiledPIDController(ARM.ARM_POS_PID[0], ARM.ARM_POS_PID[1], ARM.ARM_POS_PID[2], ARM.ARM_CONSTRAINTS);
  private PIDController velPid = new PIDController(ARM.ARM_VEL_PID[0], ARM.ARM_VEL_PID[1], ARM.ARM_VEL_PID[2]);
  private ArmFeedforward ff = new ArmFeedforward(ARM.ARM_FF[0], ARM.ARM_FF[1], ARM.ARM_FF[2]);

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

    SmartDashboard.putNumber("arm ks", 0.1);
    SmartDashboard.putNumber("arm kg", 0.5);
    SmartDashboard.putNumber("arm kv", 2);

    SmartDashboard.putData("arm pPid", pid);
    SmartDashboard.putData("arm vPid", velPid);
  }

  public void setControlMode(ArmControlMode mode) {
    pid.reset(getPosition(), getVelocity());
    velPid.reset();
    /* A velocity goal carrying over to a position goal (or vice versa) would be bad */
    setGoal(0);
    controlMode = mode;
  }

  public double getGoal() {
    return goal;
  }

  public void setGoal(double newGoal) {
    goal = newGoal;
  }

  // [Possibly a] HACK: Manually wrap absolute encoder position if dP > 2 as there are no apis to do this for us
  public double getPosition() {
    var nakedEncoderOutput = encoder.getPosition();
    if (nakedEncoderOutput - lastNakedEncoderOutput > 2) {
      iHateThis -= Math.PI * 2;
    } else if (nakedEncoderOutput - lastNakedEncoderOutput < -2) {
      iHateThis += Math.PI * 2;
    }
    lastNakedEncoderOutput = nakedEncoderOutput;
    return nakedEncoderOutput - ARM.ANGLE_OFFSET + iHateThis;
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  private void configure() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
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
    double ks = SmartDashboard.getNumber("arm ks", 0);
    double kg = SmartDashboard.getNumber("arm kg", 0);
    double kv = SmartDashboard.getNumber("arm kv", 0);

    ff = new ArmFeedforward(ks, kg, kv);

    var pose = getPosition();
    var vel = getVelocity();


    SmartDashboard.putNumber("Arm Pose", pose);
    SmartDashboard.putNumber("arm vel", vel);
    SmartDashboard.putNumber("Arm Goal", goal);
    SmartDashboard.putBoolean("Arm Control Mode == POS", controlMode == ArmControlMode.POS);
    if (controlMode == ArmControlMode.POS) {
      double pidval = pid.calculate(pose, goal);
      double ffval = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity); // ff wants 0 parallel to floor in pos x
      setVoltage(ffval + pidval);
      SmartDashboard.putNumber("arm ffval", ffval);
      SmartDashboard.putNumber("pidval", pidval);
    } else if (controlMode == ArmControlMode.VEL) {
      if ((getPosition() > Units.degreesToRadians(0) && goal > 0)
          || (getPosition() < -Units.degreesToRadians(180) && goal < 0)) {
        setVoltage(0);
        return;
      }
      double ffval = ff.calculate(pose, goal);
      double pidval = velPid.calculate(vel, goal);
      setVoltage(ffval + pidval);
    }
  }
}
