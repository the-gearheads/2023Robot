// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import frc.robot.util.AdditionalMathUtils;
import frc.robot.util.Gyroscope;

public class SwerveSubsystem extends SubsystemBase {

  Translation2d[] modulePositions = {Drivetrain.FL_POS, Drivetrain.FR_POS, Drivetrain.RL_POS, Drivetrain.RR_POS};
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modulePositions);
  public SwerveModuleIO[] modules;
  public SwerveModuleInputsAutoLogged[] lastInputs;
  Gyroscope gyro = new Gyroscope(SPI.Port.kMXP, true);
  SwerveDrivePoseEstimator odometry;
  Field2d field = new Field2d();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(SwerveModuleIO... modules) {
    this.modules = modules;
    gyro.reset();

    /* Get module states to pass to odometry */
    updateInputs();
    var states = getPositionsFromInputs(lastInputs);
    odometry = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), states, new Pose2d());

    zeroEncoders();

    /* Initialize SmartDashboard values */
    SmartDashboard.putBoolean("/Swerve/DesaturateWheelSpeeds", false);

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
    var speeds = getChassisSpeeds();
    Logger.getInstance().recordOutput("/Swerve/ChassisSpeeds/Vx", speeds.vxMetersPerSecond);
    Logger.getInstance().recordOutput("/Swerve/ChassisSpeeds/Vy", speeds.vyMetersPerSecond);
    Logger.getInstance().recordOutput("/Swerve/ChassisSpeeds/Rot", speeds.omegaRadiansPerSecond);

    var states = getStatesFromInputs(lastInputs);
    Logger.getInstance().recordOutput("/Swerve/ModuleStates", states);
  }

  /**
   * Return SwerveModulePositions for all SwerveModuleInputs
   * @param inputs SwerveModuleInputs containing velocities and angles
   * @return SwerveModulePositions containing velocities and angles
   */
  public SwerveModulePosition[] getPositionsFromInputs(SwerveModuleInputsAutoLogged[] inputs) {
    SwerveModulePosition states[] = new SwerveModulePosition[inputs.length];
    for (int i = 0; i < inputs.length; i++) {
      states[i] = new SwerveModulePosition(inputs[i].drivePosition, Rotation2d.fromRadians(inputs[i].steerAngle));
    }
    return states;
  }

  /**
   * Return SwerveModuleStates for all SwerveModuleInputs
   * @param inputs SwerveModuleInputs containing velocities and angles
   * @return SwerveModuleStates containing velocities and angles
   */
  public SwerveModuleState[] getStatesFromInputs(SwerveModuleInputsAutoLogged[] inputs) {
    SwerveModuleState states[] = new SwerveModuleState[inputs.length];
    for (int i = 0; i < inputs.length; i++) {
      states[i] = new SwerveModuleState(inputs[i].driveVelocity, Rotation2d.fromRadians(inputs[i].steerAngle));
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

  private ChassisSpeeds getChassisSpeeds() {
    var states = getStatesFromInputs(lastInputs);
    return kinematics.toChassisSpeeds(states);
  }

  /**
   * Drives the robot at the specified speeds
   * @param speeds Speeds to go
   */
  public void drive(ChassisSpeeds speeds) {
    var states = kinematics.toSwerveModuleStates(speeds);
    if(SmartDashboard.getBoolean("/Swerve/DesaturateWheelSpeeds", true)) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, getChassisSpeeds(), Drivetrain.MAX_MODULE_SPEED, Drivetrain.MAX_TRANSLATIONAL_SPEED, Drivetrain.MAX_ROTATIONAL_SPEED);
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
    odometry.resetPosition(gyro.getRotation2d(), getPositionsFromInputs(lastInputs), pose);
  }

  /* Sets all modules to zero degrees */
  public void zeroAngles() {
    for(var module: modules) {
      module.setAngle(new Rotation2d(0));
    }
  }

  /* Sets all modules into an X formation (and stops driving) */
  public void setX() {
    setStates(new SwerveModuleState[] {
      // FL
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      // FR
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      // RL
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      // RR
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
    });
  }

  /* Sets volts in a way that makes it drive like a differential drive */
  public void setVolts(double leftVolts, double rightVolts) {
    for(int i = 0; i < modules.length; i++) {
      // If i is divisible by 2, it is on the left (because order is FL, FR, RL, RR). 0 is divisible by 2 in this implementation.
      if(i % 2 == 0) {
        modules[i].setVoltage(leftVolts);
      } else {
        modules[i].setVoltage(rightVolts);
      }
    }
  }

  /* Gets drive motor voltages in a way that would make sense for a differential drive */
  public double[] getDiffVoltages() {
    return new double[] {lastInputs[0].driveAppliedVolts, lastInputs[1].driveAppliedVolts};
  }

  /* Gets drive motor positions in a way that would make sense for a differential drive */
  public double[] getDiffPositions() {
    return new double[] { lastInputs[0].drivePosition, lastInputs[1].drivePosition};
  }
    
  /* Gets drive motor velocities in a way that would make sense for a differential drive */
  public double[] getDiffVelocities() {
    return new double[] { lastInputs[0].driveVelocity, lastInputs[1].driveVelocity};
  }

  /* Returns Z axis rotation speed in degrees per second */
  public double getAngularVel() {
    return gyro.getRate();
  }
  
  /**
   * Gets current odometry pose
   * @return Current robot pose, as reported by odometry
   */
  public Pose2d getPose(){ 
      return odometry.getEstimatedPosition();
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

  public void updateVisionMeasurement(Pose2d visionRobotPos, double timestamp){
    // visionRobotPos=new Pose2d(visionRobotPos.getTranslation(), gyro.getRotation2d());
    odometry.addVisionMeasurement(visionRobotPos, timestamp);
    SmartDashboard.putString("Vision/Vision Estimated Pos", AdditionalMathUtils.pos2dToString(visionRobotPos, 2));
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
