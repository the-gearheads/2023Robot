// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;
import org.opencv.core.Mat;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  private double goal = 0;
  private CANSparkMax motor = new CANSparkMax(0, null);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private PIDController pid = new PIDController(0, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);

  public enum ArmControlMode{
    VEL, POS;
  }
  public ArmControlMode controlMode;
  
  public Arm() {
    controlMode=ArmControlMode.VEL;
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

  public double getWrappedPosition() {
    double currentPose = getPosition();  // any number
    double wrappedPose = currentPose;
    wrappedPose%=2*Math.PI;
    wrappedPose=wrappedPose>0?wrappedPose:wrappedPose+2*Math.PI; // wil be 0-360, robot wont go backward if goal is 359 and pos is 1
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run4
    double wrappedPose = getWrappedPosition();
    switch(controlMode){
      case POS:
      {
        double pidval = pid.calculate(wrappedPose, goal);
        double ffval = ff.calculate(wrappedPose+Math.PI/2, 0);  // ff wants 0 parallel to floor in pos x
        motor.setVoltage(pidval + ffval);
      }
        break;
      case VEL:
      {
        double ffval = ff.calculate(wrappedPose+Math.PI/2, goal);
        motor.setVoltage(ffval);
      }
        break;
    }
  }
}
