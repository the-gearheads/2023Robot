package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleSim extends SwerveModule {

  private double simRotDistance = 0;
  private double simDriveDistance = 0;

  private FlywheelSim rotSim, driveSim;

  public SwerveModuleSim(int driveId, int steerId, Rotation2d angleOffset, boolean invertSteer, int id, String description) {
    super(driveId, steerId, angleOffset, invertSteer, id, description);
    rotSim = steer.getSim();
    driveSim = drive.getSim();
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    super.updateInputs(inputs);
    // Now to do sim stuff
    double dt = 0.02;
    rotSim.setInputVoltage(steer.getAppliedVolts());
    driveSim.setInputVoltage(drive.getAppliedVolts());
    rotSim.update(dt);
    driveSim.update(dt);

    simRotDistance += rotSim.getAngularVelocityRPM() * 360 * dt;
    steer.setSimPosition(simRotDistance);
    steer.setSimVelocity(rotSim.getAngularVelocityRPM() * 360);

    simDriveDistance += driveSim.getAngularVelocityRPM() * drive.conversionFactor * dt;
    drive.simSetPosition(simDriveDistance);
    drive.simSetVelocity(driveSim.getAngularVelocityRPM() * drive.conversionFactor);

  }

}
