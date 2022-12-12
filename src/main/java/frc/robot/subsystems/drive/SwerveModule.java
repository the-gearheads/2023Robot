package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.drive.motors.CIMSteer;
import frc.robot.subsystems.drive.motors.NEODrive;

public class SwerveModule implements SwerveModuleIO, Sendable  {

  protected NEODrive drive;
  protected CIMSteer steer;
  
  public int id;
  public String description;
  private Rotation2d targetAngle = new Rotation2d();
  private double targetVelocity;

  public SwerveModule(int driveId, int steerId, Rotation2d angleOffset, String description, boolean invertSteer) {
    drive = new NEODrive(driveId, !invertSteer);
    steer = new CIMSteer(steerId, angleOffset);
    id = driveId;
    this.description = description;
  }

  public void zeroEncoders() {
    drive.zeroEncoders();
    steer.zeroEncoders();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(drive.getVelocity(), Rotation2d.fromDegrees(steer.getAngle()));
  }

  public void setState(SwerveModuleState state) {
    targetVelocity = state.speedMetersPerSecond;
    targetAngle = state.angle;
  }

  public void setPIDConstants(double kF, double kP, double kI, double kD){
    steer.setPIDConstants(kF, kP, kI, kD);
  }

  /* Called every periodic() */
  public void updateInputs(SwerveModuleInputs inputs) {
    /* Set our target speeds and angles */
    steer.setAngle(targetAngle.getDegrees());
    drive.setSpeed(targetVelocity);

    inputs.description = this.description;
    inputs.currentAngle = steer.getAngle();
    inputs.driveAppliedVolts = drive.getAppliedVolts();
    inputs.drivePosition = drive.getPosition();
    inputs.driveVelocity = drive.getVelocity();
    inputs.steerAppliedVolts = steer.getAppliedVolts();
    inputs.steerVelocity = steer.getVelocity();
    inputs.targetAngle = this.targetAngle.getDegrees();
    inputs.targetVelocity = this.targetVelocity;
  }

  public String getDescription() {
    return this.description;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("description", ()->{return this.description;}, null);
    builder.addDoubleProperty("currentAngle", ()->{return steer.getAngle();}, null);
    builder.addDoubleProperty("driveAppliedVolts", ()->{return drive.getAppliedVolts();}, null);
    builder.addDoubleProperty("drivePosition", ()->{return drive.getPosition();}, null);
    builder.addDoubleProperty("driveVelocity", ()->{return drive.getVelocity();}, null);
    builder.addDoubleProperty("steerAppliedVolts", ()->{return steer.getAppliedVolts();}, null);
    builder.addDoubleProperty("steerVelocity", ()->{return steer.getVelocity();}, null);
    builder.addDoubleProperty("targetAngle", ()->{return targetAngle.getDegrees();}, null);
    builder.addDoubleProperty("targetVelocity", ()->{return targetVelocity;}, null);
  }
}
