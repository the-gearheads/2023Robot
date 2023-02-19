// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ARM_PLOT;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.util.sim.Cone;
import frc.robot.util.sim.MechRootWrapper;

/** Add your docs here. */
public class WristSim extends Wrist {
  private ArmSim armSim;
  private double simVolts;

  public WristSim(ArmSim arm) {
    super(arm);
    this.armSim = arm;

  simPivot = new MechRootWrapper(this.armSim.m_mech2d, "WristPivot", 0, 0);
  simLig = simPivot.append(new MechanismLigament2d("Wrist", ARM_PLOT.WRIST_LENGTH, Units.radiansToDegrees(sim.getAngleRads()), 10.0,
  new Color8Bit(Color.kBlue)));
  simCone = new Cone(simPivot, new Pose2d(5, 0, new Rotation2d(180)));;
  }

  @Override
  public void setVoltage(double volts) {
    this.simVolts = volts;
  }

  @Override
  public double getPosition() {
    return sim.getAngleRads();
  }

  public void simulationPeriodic() {
    double armPos = armSim.getPosition();
    double pivot_x = ARM_PLOT.ARM_PIVOT_X + (armSim.m_arm.getLength()) * Math.cos(armPos);
    double pivot_y = ARM_PLOT.ARM_PIVOT_Y + (armSim.m_arm.getLength()) * Math.sin(armPos);
    simPivot.setPosition(pivot_x, pivot_y);
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

  //Setting up the Scenery:
  private final DCMotor simMotor = DCMotor.getNEO(1);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(simMotor, ARM_PLOT.WRIST_REDUCTION, SingleJointedArmSim.estimateMOI(ARM_PLOT.WRIST_LENGTH, ARM_PLOT.WRIST_MASS),
          ARM_PLOT.WRIST_LENGTH, Units.degreesToRadians(-1e10), Units.degreesToRadians(1e10), true, VecBuilder.fill(0.001) // Add noise with a std-dev of 1 tick
      );

  private final MechRootWrapper simPivot;
  private final MechanismLigament2d simLig;
  private final Cone simCone;
}
