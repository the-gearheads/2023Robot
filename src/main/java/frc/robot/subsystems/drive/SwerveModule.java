package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.motors.CIMSteer;
import frc.robot.subsystems.drive.motors.NEODrive;

public class SwerveModule {

  private NEODrive drive;
  private CIMSteer steer;

  SwerveModule(int driveId, int steerId) {
    drive = new NEODrive(driveId);
    steer = new CIMSteer(steerId);
  }

  public double getAngle() {
    return steer.getAngle();
  }
  
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getAngle());
  }

  public double getPosition() {
    return drive.getPosition();
  }

  public double getVelocity() {
    return drive.getVelocity();
  }

  public void zeroEncoders() {
    drive.zeroEncoders();
    steer.zeroEncoders();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getRotation2d());
  }

  public void setState(SwerveModuleState state) {
    drive.setSpeed(state.speedMetersPerSecond);
    steer.setAngle(state.angle.getDegrees());
  }
}
