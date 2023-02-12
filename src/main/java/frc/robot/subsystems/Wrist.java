// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private double goal;
  private CANSparkMax motor = new CANSparkMax(20, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private PIDController pid = new PIDController(0, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
  private Arm arm;


  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  private static final double m_armReduction = 200;
  private static final double m_armMass = 8.0; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(30);
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(m_armGearbox, m_armReduction, SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
          m_armLength, Units.degreesToRadians(-1e10), Units.degreesToRadians(1e10), true, VecBuilder.fill(0.001) // Add noise with a std-dev of 1 tick
      );

  private final MechanismRoot2d m_armPivot;
  private final MechanismLigament2d m_arm;
  private double simVolts;

  public void simulationPeriodic() {
    double pivot_x=30+(arm.m_arm.getLength())*Math.cos(arm.m_arm.getAngle());
    double pivot_y=30+(arm.m_arm.getLength())*Math.sin(arm.m_arm.getAngle());
    m_armPivot.setPosition(pivot_x,pivot_y);
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(this.simVolts);

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // SimBattery estimates loaded battery voltages
    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    SmartDashboard.putNumber("Wrist Vel", m_armSim.getVelocityRadPerSec());
  }


  public Wrist(Arm arm) {
    this.arm = arm;
    configure();

    m_armPivot= arm.m_mech2d.getRoot("WristPivot", 30, 30);
    m_arm=m_armPivot.append(new MechanismLigament2d("Wrist", 10.0, Units.radiansToDegrees(m_armSim.getAngleRads()),10.0,new Color8Bit(Color.kBlue)));
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
    wrappedPose=wrappedPose>0?wrappedPose:wrappedPose+2*Math.PI; // wil be 0-2*Math.PI, robot wont go backward if goal is 359 and pos is 1
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
    updateGoal();

    // This method will be called once per scheduler run
    double currentPose = getPosition();
    // double closerGoal = getClosestGoal(goal);
    double pidval = pid.calculate(currentPose, goal);
    double ffval = ff.calculate(currentPose, 0);
    motor.setVoltage(pidval + ffval);
    this.simVolts=pidval+ffval;
  }

  public double getClosestGoal(double goal) {
    double pose = getPosition();
    double difference = goal - pose;
    double mod = difference % (2*Math.PI);
    if (Math.abs(mod) <= Math.PI/2) {
        return pose + mod;
    } else {
        if (difference < 0) {
            return pose + mod + 2*Math.PI;
        } else {
            return pose + mod - 2*Math.PI;
        }
    }
}

  public void updateGoal(){    // check what range the arm is in and set the wrist accordingly
    double armWrappedPos = arm.getWrappedPosition();
    for(WristState wristState : WristState.values()){
      if(wristState.inRange(armWrappedPos)){
        double goal = wristState.getWristGoal(armWrappedPos);
        setGoal(goal);
        break;
      }
    }
  }
}
