// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.Gyroscope;

public class SwerveSubsystem extends SubsystemBase {

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drivetrain.FL_POS, Drivetrain.FR_POS, Drivetrain.RL_POS,
      Drivetrain.RR_POS);
  public SwerveModule[] modules = {
      new SwerveModule(Drivetrain.FL_DRIVE_ID, Drivetrain.FL_STEER_ID, Drivetrain.FL_OFFSET, "FL", true),
      new SwerveModule(Drivetrain.FR_DRIVE_ID, Drivetrain.FR_STEER_ID, Drivetrain.FR_OFFSET, "FR", true),
      new SwerveModule(Drivetrain.RL_DRIVE_ID, Drivetrain.RL_STEER_ID, Drivetrain.RL_OFFSET, "RL", false),
      new SwerveModule(Drivetrain.RR_DRIVE_ID, Drivetrain.RR_STEER_ID, Drivetrain.RR_OFFSET, "RR", false)
  };
  Gyroscope gyro = new Gyroscope(SPI.Port.kMXP, true);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0));
  Field2d field = new Field2d();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    gyro.reset();
    zeroEncoders();

    /* Initialize SmartDashboard values */
    SmartDashboard.putBoolean("/Swerve/ScaleWheelSpeed", false);
    SmartDashboard.putBoolean("/Swerve/UseOptimizedOptimize", false);
    SmartDashboard.putNumber("/Swerve/ShiftWindow", 0.3);
    SmartDashboard.putBoolean("/Swerve/PerformOptimizations", false);

    SmartDashboard.putData(field);

    SmartDashboard.putData("xPid", Drivetrain.Auton.X_PID);
    SmartDashboard.putData("yPid", Drivetrain.Auton.Y_PID);
    SmartDashboard.putData("rotPid", Drivetrain.Auton.ROT_PID);

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
    
    field.setRobotPose(getPose());
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


  public void setStatesOptimized(SwerveModuleState[] states) {
    if(SmartDashboard.getBoolean("/Swerve/PerformOptimizations", true)){
      states = performOptimizations(states);
    }
    setStates(states);
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

  public Pose2d getPose(){ 
      return odometry.getPoseMeters();
  }
  private double windowShiftScalingFactor = 0.3;

  /* Very experimental */
  public SwerveModuleState[] optimizedOptimize(SwerveModuleState[] currentStates, SwerveModuleState[] targetStates) {
    /* First, average the direction that all modules will take (normally) */
    double averageTargetAngle = 0;
    for (int i = 0; i < currentStates.length; i++) {
      /* If the change is greater than 90 degrees, we would normally flip */
      double delta = targetStates[i].angle.minus(currentStates[i].angle).getDegrees();
      if (Math.abs(delta) > 90) {
        averageTargetAngle += targetStates[i].angle.rotateBy(Rotation2d.fromDegrees(180)).getDegrees();
      } else {
        /* Don't flip */
        averageTargetAngle += targetStates[i].angle.getDegrees();
      }
    }
    averageTargetAngle /= currentStates.length;

    /* Now let's do it for real */
    SwerveModuleState[] finalStates = new SwerveModuleState[currentStates.length];

    for (int i = 0; i < currentStates.length; i++) {
      /* FLIP */
      double delta = targetStates[i].angle.minus(currentStates[i].angle).getDegrees();
      if (Math.abs(delta) > 90 + (averageTargetAngle * windowShiftScalingFactor)) {
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

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, boolean stopWhenDone) {
  // in your code that will be used by all path following commands.
  return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
            this.setPose(traj.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(
          traj, 
          this::getPose, // Pose supplier
          this.kinematics, // SwerveDriveKinematics
          (PIDController)SmartDashboard.getData("xPid"), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          (PIDController)SmartDashboard.getData("yPid"), // Y controller (usually the same values as X controller)
          (PIDController)SmartDashboard.getData("rotPid"), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          this::setStatesOptimized, // Module states consumer
          this // Requires this drive subsystem
      ),
      new InstantCommand(() -> {
          if(stopWhenDone) {
            drive(new ChassisSpeeds(0, 0, 0));
          }
      })
  );
}
public Command followTrajectoriesCommand(ArrayList<PathPlannerTrajectory> trajectories, boolean stopWhenDone) {
  // in your code that will be used by all path following commands.
  SequentialCommandGroup fullCommand = new SequentialCommandGroup(new InstantCommand(() -> {
    // Reset odometry for the first path you run during auto
        // this.setPose(trajectories.get(0).getInitialHolonomicPose());
    }));
  for(PathPlannerTrajectory trajectory: trajectories){
    fullCommand=fullCommand.andThen(followTrajectoryCommand(trajectory, false, false));
  }
  return fullCommand;
}
}
