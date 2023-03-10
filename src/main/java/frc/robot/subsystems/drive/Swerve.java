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
import edu.wpi.first.math.Matrix;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;

public class Swerve extends SubsystemBase {

  Translation2d[] modulePositions = {DRIVE.FL_POS, DRIVE.FR_POS, DRIVE.RL_POS, DRIVE.RR_POS};
  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modulePositions);
  public SwerveModuleIO[] modules;
  public SwerveModuleInputsAutoLogged[] lastInputs;
  public GyroIOInputsAutoLogged lastGyroInputs;
  public GyroIO gyro;
  SwerveDrivePoseEstimator odometry;
  /* Regular odometry object so we can compare vision-aided odometry with regular odometry in telemetry */
  SwerveDriveOdometry wheelOdometry;
  Field2d field = new Field2d();
  Runnable resetBuffer = () -> {
  };
  private Rotation2d gyroOffset;

  /** Creates a new SwerveSubsystem. */
  public Swerve(GyroIO gyro, SwerveModuleIO... modules) {
    this.gyro = gyro;

    this.modules = modules;
    /* Get module states to pass to odometry */
    updateInputs();

    this.odometry = new SwerveDrivePoseEstimator(kinematics, getRotation(), getModulePositions(), new Pose2d());
    this.wheelOdometry = new SwerveDriveOdometry(kinematics, getRotation(), getModulePositions());
    zeroEncoders();

    /* Initialize SmartDashboard values */
    SmartDashboard.putBoolean("Swerve/DesaturateWheelSpeeds", false);

    SmartDashboard.putData("Swerve/swerve field", field);

    SmartDashboard.putData("Swerve/xPid", AUTON.X_PID);
    SmartDashboard.putData("Swerve/yPid", AUTON.Y_PID);
    SmartDashboard.putData("Swerve/rotPid", AUTON.ROT_PID);
  }

  @Override
  public void periodic() {
    updateInputs();
    odometry.update(getRotation(), getPositionsFromInputs(lastInputs));
    wheelOdometry.update(getRotation(), getPositionsFromInputs(lastInputs));
    log();
  }

  public void log(){
    field.setRobotPose(getPose());
    field.getObject("Wheels").setPose(wheelOdometry.getPoseMeters());

    Logger.getInstance().recordOutput("Swerve/Field/Robot", getPose());
    Logger.getInstance().recordOutput("Swerve/Field/Wheels", wheelOdometry.getPoseMeters());
    Logger.getInstance().recordOutput("Swerve/Field/Vision", field.getObject("Vision").getPose());

    var speeds = getChassisSpeeds();
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/Vx", speeds.vxMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/Vy", speeds.vyMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/ChassisSpeeds/Rot", speeds.omegaRadiansPerSecond);

    var states = getStatesFromInputs(lastInputs);
    Logger.getInstance().recordOutput("Swerve/CurrentModuleStates", states);

    var commandName = this.getCurrentCommand() == null?"None" : this.getCurrentCommand().getName();
    Logger.getInstance().recordOutput("Swerve/Current Command", commandName);
  }

  public void simulationPeriodic() {
    /* Sim replay is in sim but this doesn't apply to that */
    if (Constants.getMode() != Constants.RobotMode.SIM)
      return;

    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    gyro.setRate(chassisSpeeds.omegaRadiansPerSecond);
  }

  /* Teleop-Drive Related Methods */
  /**
   * Sets all swerve modules to have the specified velocities and angles
   * 
   * @param states
   */
  public void setStates(SwerveModuleState[] states) {
    Logger.getInstance().recordOutput("Swerve/TargetModuleStates", states);
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
    Logger.getInstance().recordOutput("Swerve/DesiredSpeeds/Vx", speeds.vxMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/DesiredSpeeds/Vy", speeds.vyMetersPerSecond);
    Logger.getInstance().recordOutput("Swerve/DesiredSpeeds/Rot", speeds.omegaRadiansPerSecond);
    var states = kinematics.toSwerveModuleStates(speeds);
    if (SmartDashboard.getBoolean("Swerve/DesaturateWheelSpeeds", true)) {
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
    setPose(new Pose2d());
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
    this.gyroOffset = pose.getRotation().minus(getRotation());
    odometry.resetPosition(getRotation(), getPositionsFromInputs(lastInputs), pose);
    wheelOdometry.resetPosition(getRotation(), getPositionsFromInputs(lastInputs), pose);
    resetBuffer.run();
  }

  public void fuseVisionEstimate(Pose2d visionRobotPos, double timestamp) {
    odometry.addVisionMeasurement(visionRobotPos, timestamp);
    field.getObject("Vision").setPose(visionRobotPos);
  }

  public void fuseVisionEstimate(Pose2d visionRobotPos, double timestamp, Matrix<N3, N1> stDevs) {
    odometry.addVisionMeasurement(visionRobotPos, timestamp, stDevs);
    field.getObject("Vision").setPose(visionRobotPos);
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
      var innerTraj = traj;
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
        innerTraj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
      }
      
      // field.getObject("CurrentTrajectory").setTrajectory(innerTraj);
      Logger.getInstance().recordOutput("Swerve/Field/CurrentTrajectory", innerTraj);
    }), new PPSwerveControllerCommand(traj, this::getPose, // Pose supplier
        this.kinematics, // SwerveDriveKinematicsSS
        (PIDController) SmartDashboard.getData("Swerve/xPid"), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        (PIDController) SmartDashboard.getData("Swerve/yPid"), // Y controller (usually the same values as X controller)
        (PIDController) SmartDashboard.getData("Swerve/rotPid"), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        this::setStates, // Module states consumer
        true, this // Requires this drive subsystem
    ), new InstantCommand(() -> {
      if (stopWhenDone) {
        drive(new ChassisSpeeds(0, 0, 0));
      }
      // field.getObject("CurrentTrajectory").setTrajectory(new Trajectory());
      Logger.getInstance().recordOutput("Swerve/Field/CurrentTrajectory", new Trajectory());
    }));
  }

    /**
   * Generates a command that will follow a given trajectory
   * 
   * @param traj The trajectory to follow
   * @param isFirstPath Reset odometry to starting position if first path
   * @param stopWhenDone Append a command to stop the robot if done
   * @return
   */
  public Command silentFollowTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, boolean stopWhenDone) {
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
        var allianceTraj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
        field.getObject("CurrentTrajectory").setTrajectory(allianceTraj);
        Logger.getInstance().recordOutput("Swerve/Field/CurrentTrajectory", allianceTraj);
      }
    }), new PPSwerveControllerCommand(traj, this::getPose, // Pose supplier
        this.kinematics, // SwerveDriveKinematicsSS
        (PIDController) SmartDashboard.getData("Swerve/xPid"), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        (PIDController) SmartDashboard.getData("Swerve/yPid"), // Y controller (usually the same values as X controller)
        (PIDController) SmartDashboard.getData("Swerve/rotPid"), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        this::setStates, // Module states consumer
        true // Requires this drive subsystem
    ), new InstantCommand(() -> {
      if (stopWhenDone) {
        drive(new ChassisSpeeds(0, 0, 0));
      }
      field.getObject("CurrentTrajectory").setTrajectory(new Trajectory());
      Logger.getInstance().recordOutput("Swerve/Field/CurrentTrajectory", new Trajectory());
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
    Rotation2d startHeading = endPose.getTranslation().minus(startPose.getTranslation()).getAngle();
    Rotation2d endHeading = startHeading;
    // .rotateBy(Rotation2d.fromDegrees(180));

    PathPoint startPoint = new PathPoint(startPose.getTranslation(), startHeading, startPose.getRotation());// position, heading(direction of travel), holonomic rotation
    PathPoint endPoint = new PathPoint(endPose.getTranslation(), endHeading, endPose.getRotation());
    PathPlannerTrajectory traj = PathPlanner.generatePath(constraints, startPoint, endPoint);
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

    GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    gyro.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Gyro", gyroInputs);
    lastGyroInputs = gyroInputs;
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
    for (int i = 0; i < modules.length; i++) {
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
    return new double[] {lastInputs[0].drivePosition, -lastInputs[1].drivePosition};
  }

  /* Gets drive motor velocities in a way that would make sense for a differential drive */
  public double[] getDiffVelocities() {
    return new double[] {lastInputs[0].driveVelocity, -lastInputs[1].driveVelocity};
  }

  public Rotation2d getRotation() {
    return new Rotation2d(lastGyroInputs.angleRadians);
  }

  /* Returns Z axis rotation speed in degrees per second */
  public double getAngularVel() {
    return lastGyroInputs.angleRate;
  }

  /* Returns Z axis rotation speed in radians per second */
  public double getAngularVelRad() {
    return getAngularVel();
  }

  public double getCtsRawGyroAngle() {
    return getRotation().getDegrees();
  }

  public double getCtsRawGyroRad() {
    return getRotation().getRadians();
  }

  public double getCtsPoseRotRad(){
    return getRotation().plus(gyroOffset).getRadians();
  }

  public double getPitch() {
    return lastGyroInputs.pitchDegrees;
  }

  public double getRoll() {
    return lastGyroInputs.rollDegrees;
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

  public SwerveModulePosition[] getModulePositions() {
    return getPositionsFromInputs(lastInputs);
  }

  public Pose2d getWheelPose() {
    return wheelOdometry.getPoseMeters();
  }

  public void setResetBuffer(Runnable resetBuffer) {
    this.resetBuffer = resetBuffer;
  }
}
