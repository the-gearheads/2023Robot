// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.AUTON;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroSim;
import frc.robot.subsystems.drive.gyro.Gyroscope;
import frc.robot.util.AdditionalMathUtils;

public class SwerveSubsystem extends SubsystemBase {

  Translation2d[] modulePositions = {DRIVE.FL_POS, DRIVE.FR_POS, DRIVE.RL_POS, DRIVE.RR_POS};
  final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modulePositions);
  public SwerveModuleIO[] modules;
  public SwerveModuleInputsAutoLogged[] lastInputs;
  public GyroIO gyro;
  SwerveDrivePoseEstimator odometry;
  Field2d field = new Field2d();

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(SwerveModuleIO... modules) {
    if(Constants.getMode()==Constants.RobotMode.REAL){
      this.gyro= new Gyroscope(SPI.Port.kMXP, true);
    }else{
      this.gyro= new GyroSim();
    }
    this.modules = modules;
    /* Get module states to pass to odometry */
    updateInputs();
    zeroEncoders();

    /* Initialize SmartDashboard values */
    SmartDashboard.putBoolean("/Swerve/DesaturateWheelSpeeds", false);

    SmartDashboard.putData(field);

    SmartDashboard.putData("xPid", AUTON.X_PID);
    SmartDashboard.putData("yPid", AUTON.Y_PID);
    SmartDashboard.putData("rotPid", AUTON.ROT_PID);
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

  public void simulationPeriodic(){
    if(Constants.getMode()==Constants.RobotMode.SIM){
      ChassisSpeeds chassisSpeeds = getChassisSpeeds();
      double deltaAngle = chassisSpeeds.omegaRadiansPerSecond*0.02;
      Rotation2d newRot = gyro.getRotation2d().plus(Rotation2d.fromRadians(deltaAngle));
      gyro.setRotation2d(newRot);
    }
  }

  /* Teleop-Drive Related Methods */
  /**
   * Sets all swerve modules to have the specified velocities and angles
   * 
   * @param states
   */
  public void setStates(SwerveModuleState[] states) {
    Logger.getInstance().recordOutput("/Swerve/ModStates", states);
    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  /**
   * Drives the robot at the specified speeds
   * 
   * @param speeds Speeds to go
   */
  public void drive(ChassisSpeeds speeds) {
    var states = kinematics.toSwerveModuleStates(speeds);
    if (SmartDashboard.getBoolean("/Swerve/DesaturateWheelSpeeds", true)) {
      SwerveDriveKinematics.desaturateWheelSpeeds(states, getChassisSpeeds(), DRIVE.MAX_MODULE_SPEED,
          DRIVE.MAX_TRANSLATIONAL_SPEED, DRIVE.MAX_ROTATIONAL_SPEED);
    }
    setStates(states);
  }

  /* Odometry Related Methods */
  /**
   * Zeros all encoders to be at the default pose (0x, 0y, 0deg)
   */
  public void zeroEncoders() {
    gyro.zeroYaw();
    for (int i = 0; i < modules.length; i++) {
      modules[i].zeroEncoders();
    }
    // Needed to update lastInputs to be accurate
    updateInputs();
    odometry = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), getPositionsFromInputs(lastInputs),
        new Pose2d());
  }

  /**
   * Gets current odometry pose
   * 
   * @return Current robot pose, as reported by odometry
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry pose to the given position
   * 
   * @param pose Position robot is at
   */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositionsFromInputs(lastInputs), pose);
  }

  public void setVisionPose(Pose2d visionRobotPos, double timestamp) {
    odometry.addVisionMeasurement(visionRobotPos, timestamp);
    SmartDashboard.putString("Vision/Vision Estimated Pos", AdditionalMathUtils.pos2dToString(visionRobotPos, 2));
  }

  public void setVisionPose(Pose2d visionRobotPos, double timestamp, Matrix<N3, N1> stDevs) {
    odometry.addVisionMeasurement(visionRobotPos, timestamp, stDevs);
    SmartDashboard.putString("Vision/Vision Estimated Pos", AdditionalMathUtils.pos2dToString(visionRobotPos, 2));
  }

  /* Auton Related Methods */
  /**
   * Generates a command that will follow a given trajectory
   * 
   * @param traj The trajectory to follow
   * @param isFirstPath Reset odometry to starting position if first path
   * @param stopWhenDone Append a command to stop the robot if done
   * @return
   */
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, boolean stopWhenDone) {
    return new SequentialCommandGroup(new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      if (isFirstPath) {
        Pose2d initPose = traj.getInitialHolonomicPose();
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
          // Create a new state so that we don't overwrite the original
          Translation2d newTranslation = 
              new Translation2d(initPose.getX(), Constants.FIELD_CONSTANTS.WIDTH - initPose.getY());
          Rotation2d newHeading = initPose.getRotation().times(-1);
          initPose = new Pose2d(newTranslation, newHeading);
        }
        this.setPose(initPose);
      }
    }), new PPSwerveControllerCommand(traj, this::getPose, // Pose supplier
        this.kinematics, // SwerveDriveKinematics
        (PIDController) SmartDashboard.getData("xPid"), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        (PIDController) SmartDashboard.getData("yPid"), // Y controller (usually the same values as X controller)
        (PIDController) SmartDashboard.getData("rotPid"), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        this::setStates, // Module states consumer
        true,
        this // Requires this drive subsystem
    ), new InstantCommand(() -> {
      if (stopWhenDone) {
        drive(new ChassisSpeeds(0, 0, 0));
      }
    }));
  }

  /**
   * Follows an ArrayList of trajectories
   * 
   * @param trajectories List of trajectories to follow
   * @param stopWhenDone [unimplemented] stop when done following all trajectories
   * @return
   */
  public Command followTrajectoriesCommand(ArrayList<PathPlannerTrajectory> trajectories, boolean stopWhenDone) {
    Command fullCommand = new InstantCommand();
    boolean isFirstTrajectory = true;
    for (PathPlannerTrajectory trajectory : trajectories) {
      fullCommand = fullCommand.andThen(followTrajectoryCommand(trajectory, isFirstTrajectory, false));
      isFirstTrajectory = false;
    }
    return fullCommand;
  }

  public Command goTo(Pose2d endPose, PathConstraints constraints) {
    Pose2d startPose = getPose();
    Rotation2d startHeading = endPose.minus(startPose).getTranslation().getAngle();
    Rotation2d endHeading = startHeading;

    PathPoint startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());// position, heading(direction of travel), holonomic rotation
    PathPoint endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());
    PathPlannerTrajectory traj = PathPlanner.generatePath(
    constraints,
    startPoint,
    endPoint
    );
    return followTrajectoryCommand(traj, false, true);
  }

  /* Telemetry (Advantage Kit) Related Methods (also used in odometry) */
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

  /**
   * Return SwerveModulePositions for all SwerveModuleInputs
   * 
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
   * 
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

  public ChassisSpeeds getChassisSpeeds() {
    var states = getStatesFromInputs(lastInputs);
    return kinematics.toChassisSpeeds(states);
  }

  /**
   * Drives, but field relative. +X is away from the driver station.
   * 
   * @param speeds Speeds to go in each axis, field relative
   */
  public void driveFieldRelative(ChassisSpeeds speeds) {
    var robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond, getPose().getRotation());
    drive(robotOrientedSpeeds);
  }

  /* Sets all modules into an X formation (and stops driving) */
  public void setX() {
    // for(var module: modules) {
    //   module.setState(new SwerveModuleState());
    // }
    // modules[0].setAngle(Rotation2d.fromDegrees(45));
    // modules[1].setAngle(Rotation2d.fromDegrees(-45));
    // modules[2].setAngle(Rotation2d.fromDegrees(-45));
    // modules[3].setAngle(Rotation2d.fromDegrees(45));

    setStates(new SwerveModuleState[] {
        // FL
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        // FR
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        // RL
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        // RR
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))});
  }

  /* SysId Related Methods  */
  /* Sets volts in a way that makes it drive like a differential drive */
  public void setVolts(double leftVolts, double rightVolts) {
    rightVolts *= -1;
    for(int i = 0; i < modules.length; i++) {
      // If i is divisible by 2, it is on the left (because order is FL, FR, RL, RR). 0 is divisible by 2 in this implementation.
      if (i % 2 == 0) {
        modules[i].setVoltage(leftVolts);
      } else {
        modules[i].setVoltage(rightVolts);
      }
    }
  }

  /* Gets drive motor voltages in a way that would make sense for a differential drive */
  public double[] getDiffVoltages() {
    return new double[] {lastInputs[0].driveAppliedVolts, -lastInputs[1].driveAppliedVolts};
  }

  /* Gets drive motor positions in a way that would make sense for a differential drive */
  public double[] getDiffPositions() {
    return new double[] { lastInputs[0].drivePosition, -lastInputs[1].drivePosition};
  }

  /* Gets drive motor velocities in a way that would make sense for a differential drive */
  public double[] getDiffVelocities() {
    return new double[] { lastInputs[0].driveVelocity, -lastInputs[1].driveVelocity};
  }

  /* Returns Z axis rotation speed in degrees per second */
  public double getAngularVel() {
    return gyro.getRate();
  }

  /* Returns Z axis rotation speed in radians per second */
  public double getAngularVelRad() {
    return Units.degreesToRadians(gyro.getRate());
  }

  public double getContinuousGyroAngle(){
    return gyro.getRotation2d().getDegrees();
  }

  public double getContinuousGyroAngleRad(){
    return gyro.getRotation2d().getRadians();
  }

  /**
   * Sets steer pid constants for all modules
   * 
   * @param kF F(eedforward) value
   * @param kP P(roportional) PID value
   * @param kI I(ntegral) PID value
   * @param kD D(erivitive) PID value
   */
  public void setPIDConstants(double kF, double kP, double kI, double kD) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setPIDConstants(kF, kP, kI, kD);
    }
  }
}
