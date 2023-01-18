package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.motors.Neo550Steer;
import frc.robot.subsystems.drive.motors.NeoDrive;

public class SwerveModule implements SwerveModuleIO {

  protected NeoDrive drive;
  protected Neo550Steer steer;

  protected Rotation2d angleOffset;
  
  public int id;
  public String description;

  public SwerveModule(int id, int driveId, int steerId, double[] offsets, String description) {
    drive = new NeoDrive(driveId);
    steer = new Neo550Steer(steerId, offsets[1]);
    this.id = id;
    this.description = description;
    this.angleOffset = Rotation2d.fromDegrees(offsets[0]);
  }

  public void zeroEncoders() {
    drive.zeroEncoders();
  }

  private Rotation2d getAngle() {
    return steer.getAngle().minus(angleOffset);
  }

  public void setState(SwerveModuleState state) {
    /* We do offsetting before optimization so the wheel automatically gets reversed when the module is facing backwards */
    state.angle = state.angle.plus(angleOffset);
    if(SmartDashboard.getBoolean("/Swerve/PerformOptimizations", true)){
      state = SwerveModuleState.optimize(state, getAngle());
    }
    steer.setAngle(state.angle);
    drive.setSpeed(state.speedMetersPerSecond);
  }

  public void updateSteerPIDConstants(double kP, double kI, double kD, double kF){
    steer.updatePIDConstants(kP, kI, kD, kF);
  }

  /* Called every periodic() */
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.description = this.description;

    inputs.driveAppliedVolts = drive.getAppliedVolts();
    inputs.drivePosition = drive.getPosition();
    inputs.driveVelocity = drive.getVelocity();
    inputs.driveVelocitySetpoint = drive.getVelocitySetpoint();

    inputs.steerAppliedVolts = steer.getAppliedVolts();
    inputs.steerAngle = getAngle().getDegrees();
    inputs.steerVelocity = steer.getVelocity();
    inputs.steerAngleSetpoint = steer.getAngleSetpoint().getDegrees();
  }

  public String getDescription() {
    return this.description;
  }

  public String getPath() {
    return "Swerve/Wheel " + id + " (" + description + ")";
  }
}
