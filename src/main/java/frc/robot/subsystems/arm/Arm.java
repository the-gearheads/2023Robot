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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ARM;

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  private double goal = 0;
  protected CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private ProfiledPIDController pid = new ProfiledPIDController(1, 0, 0, ARM.ARM_CONSTRAINTS);
  private PIDController velPid = new PIDController(0.5, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0.1, 0.5, 3);

  public enum ArmControlMode {
    VEL, POS;
  }

  public ArmControlMode controlMode;
  private double lastPos;
  private double lastNakedEncoderOutput;
  private double iHateThis;

  public Arm() {
    controlMode = ArmControlMode.VEL;
    configure();

    SmartDashboard.putNumber("arm ks",0.1);
    SmartDashboard.putNumber("arm kg",0.5);
    SmartDashboard.putNumber("arm kv",2);

    SmartDashboard.putData("arm pPid", pid);
    SmartDashboard.putData("arm vPid", velPid);
  }

  // public double getClosestGoal(double goal) {
  //   double pose = getWrappedPosition();
  //   double difference = goal - pose;
  //   double mod = difference % (2 * Math.PI);
  //   if (Math.abs(mod) <= Math.PI) {
  //     return pose + mod;
  //   } else {
  //     if (difference < 0) {
  //       return pose + mod + 2 * Math.PI;
  //     } else {
  //       return pose + mod - 2 * Math.PI;
  //     }
  //   }
  // }

  public double getGoal() {
    return goal;
  }

  public void setGoal(double newGoal) {
    goal = newGoal;
  }

  // WHY DONT THEY SUPPORT CONTINUOUS ENCODER VALUES
  public double getPosition() {
    var nakedEncoderOutput = encoder.getPosition();
    if (nakedEncoderOutput-lastNakedEncoderOutput > 2){
      iHateThis-=Math.PI*2;
    }else if(nakedEncoderOutput - lastNakedEncoderOutput < -2){
      iHateThis+=Math.PI*2;
    }
    lastNakedEncoderOutput = nakedEncoderOutput;
    return nakedEncoderOutput - Constants.ARM.ANGLE_OFFSET + iHateThis;
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  // public double getWrappedPosition() {
  //   double currentPose = getPosition(); // any number
  //   double wrappedPose = currentPose;
  //   wrappedPose %= 2 * Math.PI;
  //   wrappedPose = wrappedPose > 0 ? wrappedPose : wrappedPose + 2 * Math.PI; // wil be 0-2*Math.PI, robot wont go backward if goal is 359 and pos is 1
  //   return wrappedPose;
  // }

  private void configure() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
    encoder.setPositionConversionFactor(2 * Math.PI);
    encoder.setVelocityConversionFactor(2 * Math.PI);
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    double ks = SmartDashboard.getNumber("arm ks",0);
    double kg = SmartDashboard.getNumber("arm kg",0);
    double kv = SmartDashboard.getNumber("arm kv",0);

    ff = new ArmFeedforward(ks, kg, kv);

    var pose = getPosition();
    var vel = getVelocity();


    SmartDashboard.putNumber("Arm Pose", pose);
    SmartDashboard.putNumber("arm vel", vel);
    SmartDashboard.putNumber("Arm Goal", goal);
    SmartDashboard.putBoolean("Arm Control Mode == POS", controlMode==ArmControlMode.POS);
    switch (controlMode) {
      case POS: {
        double pidval = pid.calculate(pose, goal);
        double ffval = ff.calculate(pid.getSetpoint().position, pid.getSetpoint().velocity); // ff wants 0 parallel to floor in pos x
        setVoltage(ffval + pidval);
        SmartDashboard.putNumber("arm ffval", ffval);
        SmartDashboard.putNumber("pidval", pidval);
      }
        break;
      case VEL: {
        pid.reset(pose, vel);
        if ((getPosition() > Units.degreesToRadians(0) && goal > 0)
            || (getPosition() < -Units.degreesToRadians(180) && goal < 0)) {
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
