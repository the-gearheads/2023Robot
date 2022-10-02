// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;

public class SwerveSubsystem extends SubsystemBase {

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drivetrain.FL_POS, Drivetrain.FR_POS, Drivetrain.RL_POS,
      Drivetrain.RR_POS);
  SwerveModule[] modules = {
      new SwerveModule(Drivetrain.FL_DRIVE_ID, Drivetrain.FL_STEER_ID),
      new SwerveModule(Drivetrain.FR_DRIVE_ID, Drivetrain.FR_STEER_ID),
      new SwerveModule(Drivetrain.RL_DRIVE_ID, Drivetrain.RL_STEER_ID),
      new SwerveModule(Drivetrain.RR_DRIVE_ID, Drivetrain.RR_STEER_ID)
  };
  AHRS gyro = new AHRS(SPI.Port.kMXP);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    gyro.reset();
    zeroEncoders();

    /* Initialize SmartDashboard values */
    SmartDashboard.putBoolean("/Swerve/ScaleWheelSpeed", true);
    SmartDashboard.putBoolean("/Swerve/UseOptimizedOptimize", true);
  }

  public void zeroEncoders() {
    gyro.zeroYaw();
    for (int i = 0; i < modules.length; i++) {
      modules[i].zeroEncoders();
    }
    odometry.resetPosition(new Pose2d(), gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), getStates());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = {};
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private void setStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public void drive(ChassisSpeeds speeds) {
    var states = kinematics.toSwerveModuleStates(speeds);
    SwerveModuleState[] optimizedStates = {};
    if (SmartDashboard.getBoolean("/Swerve/UseOptimizedOptimize", true)) {
      optimizedStates = optimizedOptimize(getStates(), states);
    } else {
      for (int i = 0; i < modules.length; i++) {
        optimizedStates[i] = SwerveModuleState.optimize(states[i], modules[i].getRotation2d());
      }
    }
    setStates(optimizedStates);
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    var robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond, getPose().getRotation());
    drive(robotOrientedSpeeds);
  }

  public void setPose(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  private double windowShiftScalingFactor = 0.3;

  /* Very experimental */
  public SwerveModuleState[] optimizedOptimize(SwerveModuleState[] currentStates, SwerveModuleState[] targetStates) {
    /* First, average the direction that all modules will take (normally) */
    double averageAngle = 0;
    for (int i = 0; i < currentStates.length; i++) {
      /* If the change is greater than 90 degrees, we would normally flip */
      if (Math.abs(targetStates[i].angle.minus(currentStates[i].angle).getDegrees()) > 90) {
        averageAngle += targetStates[i].angle.rotateBy(Rotation2d.fromDegrees(180)).getDegrees();
      } else {
        /* Don't flip */
        averageAngle += targetStates[i].angle.getDegrees();
      }
    }
    averageAngle /= currentStates.length;

    /* Now let's do it for real */
    SwerveModuleState[] finalStates = {};

    for (int i = 0; i < currentStates.length; i++) {
      /* FLIP */
      if (Math.abs(targetStates[i].angle.minus(currentStates[i].angle).getDegrees()) > 90
          + (averageAngle * windowShiftScalingFactor)) {
        finalStates[i] = new SwerveModuleState(-targetStates[i].speedMetersPerSecond,
            targetStates[i].angle.rotateBy(Rotation2d.fromDegrees(180)));
      } else {
        /* Don't flip */
        finalStates[i] = new SwerveModuleState(targetStates[i].speedMetersPerSecond, targetStates[i].angle);
      }
    }
    return finalStates;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
