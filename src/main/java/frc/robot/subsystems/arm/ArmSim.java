// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.MECH_PLOT;

public class ArmSim extends Arm {
  private double simVolts = 0;

  /** Creates a new ArmSim. */
  public ArmSim() {
    super();
    SmartDashboard.putData("Arm", m_mech2d);

    super.velPid.setP(MECH_PLOT.SIM_ARM_VEL_PID[0]);
    super.velPid.setI(MECH_PLOT.SIM_ARM_VEL_PID[1]);
    super.velPid.setD(MECH_PLOT.SIM_ARM_VEL_PID[2]);
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
  public double getPose() {
    return Units.radiansToDegrees(m_armSim.getAngleRads());
  }

  @Override
  public double getVel() {
    return Units.radiansToDegrees(m_armSim.getVelocityRadPerSec());
  }

  @Override
  public void setVoltage(double volts) {
    this.simVolts = MathUtil.clamp(volts, -12, 12);
  }

  @Override
  public double getVoltage() {
    return this.simVolts;
  }

  @Override
  public boolean sensorErrorHandler() {
    return false;
  }

  // Setting The Scenery
  private static final DCMotor m_armGearbox = DCMotor.getNEO(1);

  private static final double m_armReduction = 200;
  private static final double m_armMass = 8.0; // Kilograms
  private static final double m_armLength = Units.inchesToMeters(20);
  private static final SingleJointedArmSim m_armSim = new SingleJointedArmSim(m_armGearbox, m_armReduction,
      SingleJointedArmSim.estimateMOI(m_armLength, m_armMass), m_armLength, -1e10, 1e10, true, VecBuilder.fill(0.001) // Add noise with a std-dev of 1 tick
  );

  public static final Mechanism2d m_mech2d = new Mechanism2d(MECH_PLOT.PLOT_WIDTH, MECH_PLOT.PLOT_HEIGHT);

  public static final MechanismRoot2d m_armPivot =
      m_mech2d.getRoot("ArmPivot", MECH_PLOT.ARM_PIVOT_X, MECH_PLOT.ARM_PIVOT_Y);
  private static final MechanismLigament2d m_armTower = m_armPivot.append(
      new MechanismLigament2d("ArmTower", MECH_PLOT.ARM_TOWER_LENGTH, -90, 10.0, new Color8Bit(Color.kDarkGray)));
  public static final MechanismLigament2d m_arm = m_armPivot.append(new MechanismLigament2d("Arm", MECH_PLOT.ARM_LENGTH,
      Units.radiansToDegrees(m_armSim.getAngleRads()), 10.0, new Color8Bit(Color.kGray)));

  public static final MechanismRoot2d m_chassisPivot =
      m_mech2d.getRoot("ChassisPivot", MECH_PLOT.CHASSIS_X, MECH_PLOT.CHASSIS_Y);
  private static final MechanismLigament2d m_chassis = m_chassisPivot
      .append(new MechanismLigament2d("Chassis", MECH_PLOT.CHASSIS_LENGTH, 0, 100.0, new Color8Bit(Color.kDarkRed)));

  public static final MechanismRoot2d feederPivot = m_mech2d.getRoot("Feeder Pivot", 5, 25);
  private static final MechanismLigament2d feeder =
      feederPivot.append(new MechanismLigament2d("Feeder", 10, 0, 30.0, new Color8Bit(Color.kWhite)));

  // public final MechanismRoot2d midNodePivot = m_mech2d.getRoot("Mid Node Pivot", 50, 0);
  // private final MechanismLigament2d midNode = midNodePivot.append(new MechanismLigament2d("Mid Node", 13, 90,30.0,new Color8Bit(Color.kWhite)));

  public static final MechanismRoot2d highNodePivot = m_mech2d.getRoot("High Node Pivot", 55, 0);
  private static final MechanismLigament2d highNode =
      highNodePivot.append(new MechanismLigament2d("High Node", 25, 90, 30.0, new Color8Bit(Color.kWhite)));
}
