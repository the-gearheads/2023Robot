// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private double goal;
  private CANSparkMax motor = new CANSparkMax(10, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private PIDController pid = new PIDController(4, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0.2, 0.5, 0);
  private Arm arm;
  private double iHateThis;
  private double lastNakedEncoderOutput;

  public Wrist(Arm arm) {
    this.arm = arm;
    configure();
  }

  public double getGoal() {
    return goal;
  }

  public void setGoal(double newGoal) {
    goal = newGoal;
  }

  public double getPosition() {
    var nakedEncoderOutput = encoder.getPosition();
    if (nakedEncoderOutput - lastNakedEncoderOutput > 2) {
      iHateThis -= Math.PI * 2;
    } else if (nakedEncoderOutput - lastNakedEncoderOutput < -2) {
      iHateThis += Math.PI * 2;
    }
    lastNakedEncoderOutput = nakedEncoderOutput;
    return nakedEncoderOutput - Constants.ARM.ANGLE_OFFSET + iHateThis;
  }

  private void configure() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);
    encoder.setPositionConversionFactor(2 * Math.PI);
    encoder.setVelocityConversionFactor(2 * Math.PI);
    encoder.setZeroOffset(0);

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
    updateGoal();

    // This method will be called once per scheduler run
    double currentPose = getPosition();
    double pidval = pid.calculate(currentPose, goal);
    double ffval = ff.calculate(currentPose, 0);
    setVoltage(ffval + pidval);
  }

  public void updateGoal() { // check what range the arm is in and set the wrist accordingly
    double armPos = arm.getPosition();
    for (WristState wristState : WristState.values()) {
      if (wristState.inRange(armPos)) {
        double goal = wristState.getWristGoal(armPos);
        setGoal(goal);
        return;
      }
    }
  }
}
