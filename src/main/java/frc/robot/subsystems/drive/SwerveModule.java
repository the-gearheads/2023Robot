package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.motors.CIMSteer;
import frc.robot.subsystems.drive.motors.NEODrive;

public class SwerveModule {

  private NEODrive drive;
  private CIMSteer steer;
  
  public int id;
  public String description;
  public String folderName = "";
  private Rotation2d targetAngle = new Rotation2d();
  private double targetSpeed = 0;

  SwerveModule(int driveId, int steerId, Rotation2d angleOffset, String description, boolean invertSteer) {
    drive = new NEODrive(driveId, invertSteer);
    steer = new CIMSteer(steerId, angleOffset);
    id = driveId;
    this.description = description;
    folderName = "Wheel " + id + " (" + this.description + ")";
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
    targetSpeed = state.speedMetersPerSecond;
    targetAngle = state.angle;
  }

  public void setPIDConstants(double kF, double kP, double kI, double kD){
    steer.setPIDConstants(kF, kP, kI, kD);
  }
  public void setAngleOffset(Rotation2d angleOffset){
    steer.setAngleOffset(angleOffset);
  }
  public void periodic() {
    steer.setAngle(targetAngle.getDegrees());

    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/TargetAngle", targetAngle.getDegrees());
    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/CurrentAngle", getRotation2d().getDegrees());

    SmartDashboard.putNumber("/Swerve/Wheel " + folderName + "/TargetSpeed", targetSpeed);


    if(SmartDashboard.getBoolean("/Swerve/ScaleWheelSpeed", true)) {
       /* Scale drive wheel speed based on cosine difference */ 
       drive.setSpeed(targetSpeed * Math.cos(targetAngle.getRadians() - getRotation2d().getRadians()));
     } else {
      drive.setSpeed(targetSpeed);
     }
  }


}
