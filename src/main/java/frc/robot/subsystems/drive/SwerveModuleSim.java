package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleSim extends SwerveModule {

  private double drivePos;
  private double driveVel;
  private Rotation2d angle;

  public SwerveModuleSim(int id, int driveId, int steerId, double[] offsets, String description) {
    super(id, driveId, steerId, offsets, description);
    angle = new Rotation2d();
  }

  /* (non-Javadoc)
   * @see frc.robot.subsystems.drive.SwerveModule#setState(edu.wpi.first.math.kinematics.SwerveModuleState)
   */
  @Override
  public void setState(SwerveModuleState state) {
    super.setState(state);

    state = new SwerveModuleState(state.speedMetersPerSecond, state.angle);

    /* We do offsetting before optimization so the wheel automatically gets reversed when the module is facing backwards. Downside: the robot will not work without optimizations enabled. */
    // state = SwerveModuleState.optimize(state, steer.getAngle());

    this.angle = state.angle;
    this.driveVel = state.speedMetersPerSecond;
    this.drivePos += this.driveVel * 0.02;
  }

  /* Called every periodic() */
  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    super.updateInputs(inputs);
    inputs.drivePosition = this.drivePos;
    inputs.driveVelocity = this.driveVel;
    inputs.steerAngle = this.angle.getRadians();
  }
}
