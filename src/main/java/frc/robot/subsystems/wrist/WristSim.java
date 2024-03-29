// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.MECH_PLOT;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.util.sim.Cone;
import frc.robot.util.sim.MechRootWrapper;

/** Add your docs here. */
public class WristSim extends Wrist {
  private ArmSim armSim;
  private double simVolts;

  private static final DCMotor simMotor = DCMotor.getNEO(1);
  private static final SingleJointedArmSim sim = new SingleJointedArmSim(simMotor, MECH_PLOT.WRIST_REDUCTION,
      SingleJointedArmSim.estimateMOI(MECH_PLOT.WRIST_LENGTH, MECH_PLOT.WRIST_MASS), MECH_PLOT.WRIST_LENGTH, -1e10,
      1e10, true, VecBuilder.fill(0.001));
  private final MechRootWrapper simPivot;
  private final MechanismLigament2d simLig;
  private final Cone simCone;

  public WristSim(ArmSim arm) {
    super(arm);
    this.armSim = arm;

    simPivot = new MechRootWrapper(ArmSim.m_mech2d, "WristPivot", 0, 0);
    simLig = simPivot.append(new MechanismLigament2d("Wrist", MECH_PLOT.WRIST_LENGTH,
        Units.radiansToDegrees(sim.getAngleRads()), 10.0, new Color8Bit(Color.kBlue)));
    simCone = new Cone(simPivot, new Pose2d(5, 0, new Rotation2d(180)));

    super.pid.setP(Constants.MECH_PLOT.SIM_WRIST_PID[0]);
    super.pid.setI(Constants.MECH_PLOT.SIM_WRIST_PID[1]);
    super.pid.setD(Constants.MECH_PLOT.SIM_WRIST_PID[2]);
  }

  @Override
  public void setVoltage(double volts) {
    this.simVolts = MathUtil.clamp(volts, -12, 12);
  }

  @Override
  public double getPose() {
    var ctsPose = Units.radiansToDegrees(sim.getAngleRads()) + 180;
    var numWraps = (int) (ctsPose / 180.0);
    if (ctsPose > 0) {
      if (numWraps % 2 == 0) {
        return (ctsPose % 180.0);
      } else {
        return -180.0 + (ctsPose % 180.0);
      }
    } else {
      if (numWraps % 2 == 0) {
        return (ctsPose % 180.0);
      } else {
        return 180.0 + (ctsPose % 180.0);
      }
    }
  }

  public double getVelocity() {
    return Units.radiansToDegrees(sim.getVelocityRadPerSec());
  }

  public boolean hasSensorFault() {
    return false;
  }

  @Override
  public void sensorFaultHandler() {

  }

  public void simulationPeriodic() {
    double armPosRads = Units.degreesToRadians(armSim.getPose());
    double pivot_x = MECH_PLOT.ARM_PIVOT_X + (ArmSim.m_arm.getLength()) * Math.cos(armPosRads);
    double pivot_y = MECH_PLOT.ARM_PIVOT_Y + (ArmSim.m_arm.getLength()) * Math.sin(armPosRads);
    simPivot.setPosition(pivot_x, pivot_y);
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    sim.setInput(MathUtil.clamp(this.simVolts, -12, 12));

    // Next, we update it. The standard loop time is 20ms.
    sim.update(0.020);

    // SimBattery estimates loaded battery voltages
    // Update the Mechanism Arm angle based on the simulated arm angle
    simLig.setAngle(getPose());
    simCone.update(Rotation2d.fromDegrees(getPose()));
  }
}
