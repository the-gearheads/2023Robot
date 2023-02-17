// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmSim extends Arm {
  private double simVolts = 0;

  /** Creates a new ArmSim. */
  public ArmSim() {
    super();
    SmartDashboard.putData("Arm", m_mech2d);
  }

  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(this.simVolts);

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    SmartDashboard.putNumber("Arm Vel", m_armSim.getVelocityRadPerSec());
  }

  @Override
  public double getPosition() {
    return m_armSim.getAngleRads();
  }

  @Override
  public double getVelocity() {
    return m_armSim.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double volts) {
    this.simVolts = volts;
  }


  // Setting The Scenery
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
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90, 10.0, new Color8Bit(Color.kDarkGray)));

  public final MechanismRoot2d m_chassisPivot = m_mech2d.getRoot("ChassisPivot", 15, 2);
  private final MechanismLigament2d m_chassis =
      m_chassisPivot.append(new MechanismLigament2d("Chassis", 20, 0, 100.0, new Color8Bit(Color.kDarkRed)));

  public final MechanismRoot2d feederPivot = m_mech2d.getRoot("Feeder Pivot", 5, 25);
  private final MechanismLigament2d feeder =
      feederPivot.append(new MechanismLigament2d("Feeder", 10, 0, 30.0, new Color8Bit(Color.kWhite)));

  // public final MechanismRoot2d midNodePivot = m_mech2d.getRoot("Mid Node Pivot", 50, 0);
  // private final MechanismLigament2d midNode = midNodePivot.append(new MechanismLigament2d("Mid Node", 13, 90,30.0,new Color8Bit(Color.kWhite)));

  public final MechanismRoot2d highNodePivot = m_mech2d.getRoot("High Node Pivot", 55, 0);
  private final MechanismLigament2d highNode =
      highNodePivot.append(new MechanismLigament2d("High Node", 25, 90, 30.0, new Color8Bit(Color.kWhite)));

  public final MechanismLigament2d m_arm = m_armPivot.append(new MechanismLigament2d("Arm", 20,
      Units.radiansToDegrees(m_armSim.getAngleRads()), 10.0, new Color8Bit(Color.kGray)));
}
