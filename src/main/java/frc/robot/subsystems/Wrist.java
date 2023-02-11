// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private double goal;
  private CANSparkMax motor = new CANSparkMax(0, null);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private PIDController pid = new PIDController(0, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
  private Arm arm;

  public enum WristState{//assuming arm pointing completely upwards is 0 degrees; increasing counterclockwise; range [-pi, pi]
    UP(-90, 60), LINEAR(61, 110), FORWARD(-91, 109);
    private double max;
    private double min;

    private WristState(double min, double max){
      this.min=min;
      this.max=max;
    }

  }

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
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double currentPose = encoder.getPosition();
    double pidval = pid.calculate(currentPose, goal);
    double ffval = ff.calculate(currentPose, 0);
    motor.setVoltage(pidval + ffval);

    // check what range the arm is in and set the wrist accordingly
    double armPose = Units.radiansToDegrees(arm.getPosition()) % 360; // I don't know what this is in i wish debugging was easier with this stupid dumb roboticriov2

    if (WristState.UP.min <= armPose && armPose <= WristState.UP.max) {
      setGoal(0);
    } else if (WristState.LINEAR.min <= armPose && armPose <= WristState.LINEAR.max) {
      setGoal(armPose);
    } else if (WristState.FORWARD.min <= armPose && armPose <= WristState.FORWARD.max) {
      setGoal(90);
    }
  }
}
