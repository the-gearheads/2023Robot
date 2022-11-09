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
import frc.robot.util.Gyroscope;

public class SwerveSubsystem extends SubsystemBase {

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drivetrain.FL_POS, Drivetrain.FR_POS, Drivetrain.RL_POS,
      Drivetrain.RR_POS);
  SwerveModule[] modules = {
      new SwerveModule(Drivetrain.FL_DRIVE_ID, Drivetrain.FL_STEER_ID, Drivetrain.FL_OFFSET, "FL"),
      new SwerveModule(Drivetrain.FR_DRIVE_ID, Drivetrain.FR_STEER_ID, Drivetrain.FR_OFFSET, "FR"),
      new SwerveModule(Drivetrain.RL_DRIVE_ID, Drivetrain.RL_STEER_ID, Drivetrain.RL_OFFSET, "RL"),
      new SwerveModule(Drivetrain.RR_DRIVE_ID, Drivetrain.RR_STEER_ID, Drivetrain.RR_OFFSET, "RR")
  };
  Gyroscope gyro = new Gyroscope(SPI.Port.kMXP, true);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0));

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    gyro.reset();
    zeroEncoders();

    /* Initialize SmartDashboard values */
    SmartDashboard.putBoolean("/Swerve/ScaleWheelSpeed", true);
    SmartDashboard.putBoolean("/Swerve/UseOptimizedOptimize", true);
    SmartDashboard.putNumber("/Swerve/ShiftWindow", 0.3);
    SmartDashboard.putBoolean("/Swerve/PerformOptimizations", true);
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

    for (int i = 0; i < modules.length; i++) {
      modules[i].periodic();
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState states[] = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void setStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public SwerveModuleState[] performOptimizations(SwerveModuleState[] states) {
    windowShiftScalingFactor = SmartDashboard.getNumber("/Swerve/ShiftWindow", 0.3);
    SwerveModuleState[] optimizedStates = new SwerveModuleState[modules.length];
    if (SmartDashboard.getBoolean("/Swerve/UseOptimizedOptimize", true)) {
      optimizedStates = optimizedOptimize(getStates(), states);
    } else {
      for (int i = 0; i < modules.length; i++) {
        optimizedStates[i] = SwerveModuleState.optimize(states[i], modules[i].getRotation2d());
      }
    }
    return optimizedStates;
  }

  public void drive(ChassisSpeeds speeds) {
    var states = kinematics.toSwerveModuleStates(speeds);
    if(SmartDashboard.getBoolean("/Swerve/PerformOptimizations", true)){
    states = performOptimizations(states);
    }


    /* Somewhat redundant */
    /*
    for(int i = 0; i < optimizedStates.length; i++) {
      SmartDashboard.putNumber("/Swerve/Wheel " + modules[i].folderName + "/Speed", optimizedStates[i].speedMetersPerSecond);
      SmartDashboard.putNumber("/Swerve/Wheel " + modules[i].folderName + "/Angle", optimizedStates[i].angle.getRadians());
    }
    */
    
    setStates(states);
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
    SwerveModuleState[] finalStates = new SwerveModuleState[currentStates.length];

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


  public void setPIDConstants(double kF, double kP, double kI, double kD){
    for (int i = 0; i < modules.length; i++) {
      modules[i].setPIDConstants(kF, kP, kI, kD);
    }
  }

  public void setAngleOffsets(Rotation2d[] angleOffsets){
    for (int i = 0; i < modules.length; i++) {
      modules[i].setAngleOffset(angleOffsets[i]);
    }
  }
}
