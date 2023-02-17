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

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private double goal;
  private CANSparkMax motor = new CANSparkMax(20, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private PIDController pid = new PIDController(20, 0, 0);
  private ArmFeedforward ff = new ArmFeedforward(0, 0.005, 0);
  private Arm arm;


  private final DCMotor simMotor = DCMotor.getNEO(1);

  private static final double m_armReduction = 200;
  private static final double m_armMass = 2.0; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(30);
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(simMotor, m_armReduction, SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
          m_armLength, Units.degreesToRadians(-1e10), Units.degreesToRadians(1e10), true, VecBuilder.fill(0.001) // Add noise with a std-dev of 1 tick
      );

  private final MechRootWrapper simPivot;
  private final MechanismLigament2d simLig;
  private final Cone simCone;
  private double simVolts;

  public void simulationPeriodic() {
    double armPos = arm.getPosition();
    double pivot_x=30+(arm.m_arm.getLength())*Math.cos(armPos);
    double pivot_y=30+(arm.m_arm.getLength())*Math.sin(armPos);
    simPivot.setPosition(pivot_x,pivot_y);
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    sim.setInput(this.simVolts);

    // Next, we update it. The standard loop time is 20ms.
    sim.update(0.020);

    // SimBattery estimates loaded battery voltages
    // Update the Mechanism Arm angle based on the simulated arm angle
    simLig.setAngle(Units.radiansToDegrees(sim.getAngleRads()));
    simCone.update(new Rotation2d(sim.getAngleRads()));
    SmartDashboard.putNumber("Wrist Vel", sim.getVelocityRadPerSec());
  }


  public Wrist(Arm arm) {
    this.arm = arm;
    configure();

    simPivot= new MechRootWrapper(this.arm.m_mech2d, "WristPivot", 50, 30);
    simCone = new Cone(simPivot, new Pose2d(5,0, new Rotation2d(180)));
    simLig=simPivot.append(new MechanismLigament2d("Wrist", 10.0, Units.radiansToDegrees(sim.getAngleRads()),10.0,new Color8Bit(Color.kBlue)));
  }

  public double getGoal() {
    return goal;
  }

  public void setGoal(double newGoal) {
    goal = newGoal;
  }

  public double getPosition() {
    if (Constants.getMode() == RobotMode.SIM) {
      return sim.getAngleRads();
    }
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

  private void setVoltage(double pidval, double ffval) {
    motor.setVoltage(pidval + ffval);
    this.simVolts=pidval+ffval;
  }
  @Override
  public void periodic() {
    updateGoal();

    // This method will be called once per scheduler run
    double currentPose = getPosition();
    double pidval = pid.calculate(currentPose, goal);
    double ffval = ff.calculate(currentPose, 0);
    setVoltage(pidval, ffval);
  }

  public void updateGoal(){    // check what range the arm is in and set the wrist accordingly
    double armPos = arm.getPosition();
    for(WristState wristState : WristState.values()){
      if(wristState.inRange(armPos)){
        double goal = wristState.getWristGoal(armPos);
        setGoal(goal);
        return;
      }
    }
  }
}
