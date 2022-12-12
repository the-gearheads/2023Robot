// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleInputs;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.Gyroscope;

public class SwerveSubsystem extends SubsystemBase {

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Drivetrain.FL_POS, Drivetrain.FR_POS, Drivetrain.RL_POS,
      Drivetrain.RR_POS);
  public SwerveModuleIO[] modules;
  public SwerveModuleInputs[] lastInputs;
  Gyroscope gyro = new Gyroscope(SPI.Port.kMXP, true);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(0));
  Field2d field = new Field2d();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(SwerveModuleIO... modules) {
    this.modules = modules;
    gyro.reset();
    zeroEncoders();

    /* Initialize SmartDashboard values */
    SmartDashboard.putBoolean("/Swerve/PerformOptimizations", true);
    SmartDashboard.putBoolean("/Swerve/CoolWheelStuff", true);

    SmartDashboard.putData(field);

    SmartDashboard.putData("xPid", Drivetrain.Auton.X_PID);
    SmartDashboard.putData("yPid", Drivetrain.Auton.Y_PID);
    SmartDashboard.putData("rotPid", Drivetrain.Auton.ROT_PID);

    for(int i = 0; i < modules.length; i++) { 
      String name = "/Swerve/Wheel " + i + " (" + modules[i].getDescription() + ")";
      SmartDashboard.putData(name, (Sendable)modules[i]);
    }

  }

  /**
   * Zeros all encoders to be at the default pose (0x, 0y, 0deg)
   */
  public void zeroEncoders() {
    gyro.zeroYaw();
    for (int i = 0; i < modules.length; i++) {
      modules[i].zeroEncoders();
    }
    odometry.resetPosition(new Pose2d(), gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    /* Get inputs for each swerve module */
    SwerveModuleInputs[] inputs = new SwerveModuleInputs[modules.length];
    for (int i = 0; i < modules.length; i++) {
      inputs[i] = new SwerveModuleInputs();
      modules[i].updateInputs(inputs[i]);
    }
    lastInputs = inputs;

    odometry.update(gyro.getRotation2d(), getStatesFromInputs(inputs));
    field.setRobotPose(getPose());
  }

  /**
   * Return SwerveModuleStates for all SwerveModuleInputs
   * @param inputs SwerveModuleInputs containing velocities and angles
   * @return SwerveModuleStates containing velocities and angles
   */
  public SwerveModuleState[] getStatesFromInputs(SwerveModuleInputs[] inputs) {
    SwerveModuleState states[] = new SwerveModuleState[inputs.length];
    for (int i = 0; i < inputs.length; i++) {
      states[i] = new SwerveModuleState(inputs[i].driveVelocity, Rotation2d.fromDegrees(inputs[i].currentAngle));
    }
    return states;
  }

  /**
   * Sets all swerve modules to have the specified velocities and angles
   * @param states
   */
  public void setStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  /**
   * Like setStates, but will optimize the states before setting them if optimizations are enabled.
   * @param states States to set
   */
  public void setStatesOptimized(SwerveModuleState[] states) {
    if(SmartDashboard.getBoolean("/Swerve/PerformOptimizations", true)){
      states = optimize(states);
    }
    setStates(states);
  }

  /**
   * Optimizes swerve module states to determine whether it's worth it to flip the wheel or go the full way,
   * based on current module positions. 
   * @param states States you wish to optimize
   * @return
   */
  public SwerveModuleState[] optimize(SwerveModuleState[] states) {
    SwerveModuleState[] optimizedStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      optimizedStates[i] = SwerveModuleState.optimize(states[i], Rotation2d.fromDegrees(lastInputs[i].currentAngle));
    }
    return optimizedStates;
  }

  /**
   * Drives the robot at the specified speeds
   * @param speeds Speeds to go
   */
  public void drive(ChassisSpeeds speeds) {
    var states = kinematics.toSwerveModuleStates(speeds);
    if(SmartDashboard.getBoolean("/Swerve/PerformOptimizations", true)){
      states = optimize(states);
    }
    setStates(states);
  }

  /**
   * Drives, but field relative. +X is away from the driver station. 
   * @param speeds Speeds to go in each axis, field relative
   */
  public void driveFieldRelative(ChassisSpeeds speeds) {
    var robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond, getPose().getRotation());
    drive(robotOrientedSpeeds);
  }

  /**
   * Resets the odometry pose to the given position
   * @param pose Position robot is at
   */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Gets current odometry pose
   * @return Current robot pose, as reported by odometry
   */
  public Pose2d getPose(){ 
      return odometry.getPoseMeters();
  }

  /**
   * Sets steer pid constants for all modules
   * @param kF F(eedforward) value
   * @param kP P(roportional) PID value
   * @param kI I(ntegral) PID value
   * @param kD D(erivitive) PID value
   */
  public void setPIDConstants(double kF, double kP, double kI, double kD){
    for (int i = 0; i < modules.length; i++) {
      modules[i].setPIDConstants(kF, kP, kI, kD);
    }
  }

  /**
   * Generates a command that will follow a given trajectory
   * @param traj The trajectory to follow
   * @param isFirstPath Reset odometry to starting position if first path
   * @param stopWhenDone Append a command to stop the robot if done
   * @return
   */
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, boolean stopWhenDone) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            this.setPose(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj,
            this::getPose, // Pose supplier
            this.kinematics, // SwerveDriveKinematics
            (PIDController) SmartDashboard.getData("xPid"), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            (PIDController) SmartDashboard.getData("yPid"), // Y controller (usually the same values as X controller)
            (PIDController) SmartDashboard.getData("rotPid"), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::setStatesOptimized, // Module states consumer
            this // Requires this drive subsystem
        ),
        new InstantCommand(() -> {
          if (stopWhenDone) {
            drive(new ChassisSpeeds(0, 0, 0));
          }
        }));
  }

  /**
   * Follows an ArrayList of trajectories
   * @param trajectories List of trajectories to follow
   * @param stopWhenDone [unimplemented] stop when done following all trajectories
   * @return
   */
  public Command followTrajectoriesCommand(ArrayList<PathPlannerTrajectory> trajectories, boolean stopWhenDone) {
    Command fullCommand = new InstantCommand();
    for (PathPlannerTrajectory trajectory: trajectories) {
      fullCommand = fullCommand.andThen(followTrajectoryCommand(trajectory, false, false));
    }
    return fullCommand;
  }
}
