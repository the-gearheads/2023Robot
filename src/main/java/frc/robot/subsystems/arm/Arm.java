// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
<<<<<<< HEAD
import edu.wpi.first.math.util.Units;
=======
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
>>>>>>> a37b2f2674aa27a2b4b4a2cd24cc2a75de35c89b
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ARM;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  private double goal = 0;
  protected CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private ProfiledPIDController pid = new ProfiledPIDController(7, 0, 0, ARM.armConstraints);
  private PIDController velPid = new PIDController(3, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0.1, 0.45, 4);

  public enum ArmControlMode {
    VEL, POS;
  }

  public ArmControlMode controlMode;

  public Arm() {
    controlMode = ArmControlMode.VEL;
    configure();
  }

  public double getClosestGoal(double goal) {
    double pose = getWrappedPosition();
    double difference = goal - pose;
    double mod = difference % (2 * Math.PI);
    if (Math.abs(mod) <= Math.PI) {
      return pose + mod;
    } else {
      if (difference < 0) {
        return pose + mod + 2 * Math.PI;
      } else {
        return pose + mod - 2 * Math.PI;
      }
    }
  }

  public double getGoal() {
    return goal;
  }

  public void setGoal(double newGoal) {
    goal = newGoal;
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getWrappedPosition() {
    double currentPose = getPosition(); // any number
    double wrappedPose = currentPose;
    wrappedPose %= 2 * Math.PI;
    wrappedPose = wrappedPose > 0 ? wrappedPose : wrappedPose + 2 * Math.PI; // wil be 0-2*Math.PI, robot wont go backward if goal is 359 and pos is 1
    return wrappedPose;
  }

  private void configure() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    encoder.setPositionConversionFactor(2 * Math.PI);
    encoder.setVelocityConversionFactor(2 * Math.PI);
    encoder.setZeroOffset(0);
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    var pose = getPosition();
    var vel = getVelocity();

    switch (controlMode) {
      case POS: {
        pid.reset(pose, vel);
        double pidval = pid.calculate(pose, goal);
        double ffval = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity); // ff wants 0 parallel to floor in pos x
        setVoltage(ffval + pidval);
      }
        break;
      case VEL: {
        if ((getPosition() > Units.degreesToRadians(0) && goal > 0)
            || (getPosition() < -Units.degreesToRadians(200) && goal < 0)) {
          setVoltage(0);
          return;
        }
        double ffval = ff.calculate(pose, goal);
        double pidval = velPid.calculate(vel, goal);
        setVoltage(ffval + pidval);
      }
        break;
    }
  }
}
