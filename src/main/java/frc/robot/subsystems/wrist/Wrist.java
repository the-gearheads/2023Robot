// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import java.util.function.Function;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.arm.Arm;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private double goal;
  private CANSparkMax motor = new CANSparkMax(20, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private PIDController pid = new PIDController(20, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0, 0.005, 0);
  private Arm arm;

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
    return encoder.getPosition();
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
    updateGoal();

    // This method will be called once per scheduler run
    double currentPose = getPosition();
    double pidval = pid.calculate(currentPose, goal);
    double ffval = ff.calculate(currentPose, 0);
    setVoltage(ffval+pidval);
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
