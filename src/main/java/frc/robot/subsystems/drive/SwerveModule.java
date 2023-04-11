package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.motors.Neo550Steer;
import frc.robot.subsystems.drive.motors.NeoDrive;
import frc.robot.util.RevConfigUtils;

public class SwerveModule implements SwerveModuleIO {

  protected NeoDrive drive;
  protected Neo550Steer steer;

  protected Rotation2d angleOffset;

  private int id;
  private String description;

  public SwerveModule(int id, int driveId, int steerId, double[] offsets, String description) {
    this.id = id;
    this.description = description;
    this.angleOffset = Rotation2d.fromDegrees(offsets[0]);
    drive = new NeoDrive(driveId, getPath());
    steer = new Neo550Steer(steerId, offsets[1], getPath());
  }

  @Override
  public void zeroEncoders() {
    drive.zeroEncoders();
  }

  private Rotation2d getAngle() {
    return steer.getAngle().minus(angleOffset);
  }

  /* Directly sets module angle */
  @Override
  public void setAngle(Rotation2d newAngle) {
    steer.setAngle(newAngle.plus(angleOffset));
  }

  /* Directly sets drive motor volts, ignoring pid */
  @Override
  public void setVoltage(double volts) {
    drive.setVoltage(volts);
  }

  @Override
  public void setState(SwerveModuleState state) {
    // create deepcopy so subsequent mutations are not percieved by SwerveKinematics
    state = new SwerveModuleState(state.speedMetersPerSecond, state.angle);

    /* We do offsetting before optimization so the wheel automatically gets reversed when the module is facing backwards. Downside: the robot will not work without optimizations enabled. */
    state.angle = state.angle.plus(angleOffset);
    state = SwerveModuleState.optimize(state, steer.getAngle());

    steer.setAngle(state.angle);
    drive.setSpeed(state.speedMetersPerSecond);
  }

  @Override
  public void reconfigure() {
    RevConfigUtils.configure(steer::configure, "Swerve " + id + "Steer");
    RevConfigUtils.configure(drive::configure, "Swerve " + id + "Drive");
  }

  public void updateSteerPIDConstants(double kP, double kI, double kD, double kF) {
    steer.updatePIDConstants(kP, kI, kD, kF);
  }

  /* Called every periodic() */
  @Override
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

  @Override
  public String getDescription() {
    return this.description;
  }

  @Override
  public int getId() {
    return this.id;
  }

  @Override
  public String getPath() {
    return "Swerve/Wheel " + id + " (" + description + ")";
  }
}
