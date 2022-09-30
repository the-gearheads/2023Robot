// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;

public class SwerveSubsystem extends SubsystemBase {

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drivetrain.FL_POS, Drivetrain.FR_POS, Drivetrain.RL_POS, Drivetrain.RR_POS);
  SwerveModule[] modules = {
    new SwerveModule(Drivetrain.FL_DRIVE_ID, Drivetrain.FL_STEER_ID),
    new SwerveModule(Drivetrain.FR_DRIVE_ID, Drivetrain.FR_STEER_ID),
    new SwerveModule(Drivetrain.RL_DRIVE_ID, Drivetrain.RL_STEER_ID),
    new SwerveModule(Drivetrain.RR_DRIVE_ID, Drivetrain.RR_STEER_ID)
  };
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem() {
    gyro.reset();
    zeroEncoders();
  }


  public void zeroEncoders() {
    gyro.zeroYaw();
    for(int i = 0; i < modules.length; i++) {
      modules[i].zeroEncoders();
    }
    odometry.resetPosition(new Pose2d(), gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
