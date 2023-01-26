package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.motors.Neo550Steer;
import frc.robot.subsystems.drive.motors.NeoDrive;

public class SwerveModule implements SwerveModuleIO {

  protected NeoDrive drive;
  protected Neo550Steer steer;

  protected Rotation2d angleOffset;
  
  public int id;
  public String description;

  public SwerveModule(int id, int driveId, int steerId, double[] offsets, String description) {
    this.id = id;
    this.description = description;
    this.angleOffset = Rotation2d.fromDegrees(offsets[0]);
    drive = new NeoDrive(driveId);
    steer = new Neo550Steer(steerId, offsets[1], getPath());
  }

  public void zeroEncoders() {
    drive.zeroEncoders();
  }

  private Rotation2d getAngle() {
    return new Rotation2d(steer.getAngle().getRadians() - angleOffset.getRadians());
  }

  /* Directly sets module angle */
  public void setAngle(Rotation2d newAngle) {
    steer.setAngle(newAngle.plus(angleOffset));
  }

  /* Directly sets drive motor volts, ignoring pid */
  public void setVoltage(double volts) {
    drive.setVoltage(volts);
  }

  public void setState(SwerveModuleState state) {
    /* We do offsetting before optimization so the wheel automatically gets reversed when the module is facing backwards. Downside: the robot will not work without optimizations enabled. */
    state.angle = state.angle.plus(angleOffset);
    state = SwerveModuleState.optimize(state, steer.getAngle());

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
    inputs.steerAngle = getAngle().getRadians();
    inputs.steerVelocity = steer.getVelocity();
    inputs.steerAngleSetpoint = steer.getAngleSetpoint().getRadians();
  }

  public String getDescription() {
    return this.description;
  }

  public String getPath() {
    return "Swerve/Wheel " + id + " (" + description + ")";
  }
}
