// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;
import frc.robot.util.Gyroscope;

public class SwerveSubsystem extends SubsystemBase {

  Translation2d[] modulePositions = {Drivetrain.FL_POS, Drivetrain.FR_POS, Drivetrain.RL_POS, Drivetrain.RR_POS};
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modulePositions);
  public SwerveModuleIO[] modules;
  public SwerveModuleInputsAutoLogged[] lastInputs;
  Gyroscope gyro = new Gyroscope(SPI.Port.kMXP, true);
  SwerveDriveOdometry odometry;
  Field2d field = new Field2d();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(SwerveModuleIO... modules) {
    this.modules = modules;
    gyro.reset();

    /* Get module states to pass to odometry */
    updateInputs();
    var states = getPositionsFromInputs(lastInputs);
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), states);

    zeroEncoders();

    /* Initialize SmartDashboard values */
    SmartDashboard.putBoolean("/Swerve/PerformOptimizations", true);
    SmartDashboard.putBoolean("/Swerve/CoolWheelStuff", true);

    SmartDashboard.putData(field);

    SmartDashboard.putData("xPid", Drivetrain.Auton.X_PID);
    SmartDashboard.putData("yPid", Drivetrain.Auton.Y_PID);
    SmartDashboard.putData("rotPid", Drivetrain.Auton.ROT_PID);
  }

  /**
   * Zeros all encoders to be at the default pose (0x, 0y, 0deg)
   */
  public void zeroEncoders() {
    gyro.zeroYaw();
    for (int i = 0; i < modules.length; i++) {
      modules[i].zeroEncoders();
    }
    odometry.resetPosition(gyro.getRotation2d(), getPositionsFromInputs(lastInputs), new Pose2d());
  }

  private void updateInputs() {
    /* Get inputs for each swerve module */
    SwerveModuleInputsAutoLogged[] inputs = new SwerveModuleInputsAutoLogged[modules.length];
    for (int i = 0; i < modules.length; i++) {
      inputs[i] = new SwerveModuleInputsAutoLogged();
      modules[i].updateInputs(inputs[i]);
      Logger.getInstance().processInputs(modules[i].getPath(), inputs[i]);
    }
    lastInputs = inputs;
  }

  @Override
  public void periodic() {
    updateInputs();
    odometry.update(gyro.getRotation2d(), getPositionsFromInputs(lastInputs));
    field.setRobotPose(getPose());
    Logger.getInstance().recordOutput("/Swerve/Pose", getPose());
  }

  /**
   * Return SwerveModulePositions for all SwerveModuleInputs
   * @param inputs SwerveModuleInputs containing velocities and angles
   * @return SwerveModulePositions containing velocities and angles
   */
  public SwerveModulePosition[] getPositionsFromInputs(SwerveModuleInputsAutoLogged[] inputs) {
    SwerveModulePosition states[] = new SwerveModulePosition[inputs.length];
    for (int i = 0; i < inputs.length; i++) {
      states[i] = new SwerveModulePosition(inputs[i].drivePosition, Rotation2d.fromDegrees(inputs[i].steerAngle));
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
   * Drives the robot at the specified speeds
   * @param speeds Speeds to go
   */
  public void drive(ChassisSpeeds speeds) {
    var states = kinematics.toSwerveModuleStates(speeds);
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
    odometry.resetPosition(gyro.getRotation2d(), getPositionsFromInputs(lastInputs), pose);
  }

  /**
   * Gets current odometry pose
   * @return Current robot pose, as reported by odometry
   */
  public Pose2d getPose(){ 
      return odometry.getPoseMeters();
  }

  public double getContinuousGyroAngle(){
    return gyro.getContinuousAngle();
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
            this::setStates, // Module states consumer
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
