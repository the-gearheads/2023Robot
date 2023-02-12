// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;
import org.opencv.core.Mat;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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

public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  private double goal = 0;
  private CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private PIDController pid = new PIDController(7, 0, 0);
  private PIDController velPid = new PIDController(5, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0.2, 0.7, 4);
  private final DCMotor m_armGearbox = DCMotor.getNEO(1);

  private static final double m_armReduction = 200;
  private static final double m_armMass = 8.0; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(20);
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(m_armGearbox, m_armReduction, SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
          m_armLength, Units.degreesToRadians(-1e10), Units.degreesToRadians(1e10), true, VecBuilder.fill(0.001) // Add noise with a std-dev of 1 tick
      );

  public final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  public final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90,10.0,new Color8Bit(Color.kBlack)));
  public final MechanismLigament2d m_arm =
      m_armPivot.append(new MechanismLigament2d("Arm", 20, Units.radiansToDegrees(m_armSim.getAngleRads()),10.0,new Color8Bit(Color.kGray)));

  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(this.simVolts);

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // SimBattery estimates loaded battery voltages
    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    SmartDashboard.putNumber("Arm Vel", m_armSim.getVelocityRadPerSec());
  }

  public enum ArmControlMode {
    VEL, POS;
  }

  public ArmControlMode controlMode;
  private double simVolts = 0;

  public Arm() {
    controlMode = ArmControlMode.VEL;
    configure();
    SmartDashboard.putData("Arm Sim", m_mech2d);
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
    if (Constants.getMode() == RobotMode.SIM) {
      return m_armSim.getAngleRads();
    }
    return encoder.getPosition();
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

  private void setVoltage(double volts){
    motor.setVoltage(volts);
    if(Constants.getMode()==RobotMode.SIM){
      this.simVolts=volts;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    double pose = getPosition();
    double vel = encoder.getVelocity();

    if (Constants.getMode() == RobotMode.SIM) {
      pose = m_armSim.getAngleRads();
      vel = m_armSim.getVelocityRadPerSec();
    }
    switch (controlMode) {
      case POS: {
        double pidval = pid.calculate(pose, goal);
        double ffval = ff.calculate(pose, 0); // ff wants 0 parallel to floor in pos x
        setVoltage(ffval+pidval);
      }
        break;
      case VEL: {
        if((getPosition()>Units.degreesToRadians(45) && goal>0) || (getPosition()<-Units.degreesToRadians(225) && goal<0)){
          setVoltage(0);
          return;
        }
        double ffval = ff.calculate(pose, goal);
        double pidval = velPid.calculate(vel, goal);
        setVoltage(ffval+pidval);
      }
        break;
    }
  }
}
