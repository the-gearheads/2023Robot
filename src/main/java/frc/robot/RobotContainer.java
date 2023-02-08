// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AUTON;
import frc.robot.Constants.DRIVE;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.AutonChooser;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleSim;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should
 * be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve swerve;
  @SuppressWarnings("unused")
  private final Vision vision;
  private final AutonChooser autonChooser;

  public String readPipelineFile() {
    try {
      return Files.readString(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "streampath.json"),
          Charset.forName("UTF-8"));
    } catch (Exception e) {
      e.printStackTrace();
      return "";
    }
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default teleop command
    // Ideally, we publish this from vision-testing, but this is fine for now. In a deploy file because escaping quotation marks is ugly and annoying
    NetworkTableInstance.getDefault().getStringTopic("/VisionTestingPipeline").publish().set(readPipelineFile());
    switch (Constants.getMode()) {
      case REAL:
        SmartDashboard.putString("/Mode", "REAL");
        swerve = new Swerve(
            new Gyro(SPI.Port.kMXP, true),
            new SwerveModule(0, DRIVE.FL_IDS[0], DRIVE.FL_IDS[1], DRIVE.FL_OFFSETS, "FL"),
            new SwerveModule(1, DRIVE.FR_IDS[0], DRIVE.FR_IDS[1], DRIVE.FR_OFFSETS, "FR"),
            new SwerveModule(2, DRIVE.RL_IDS[0], DRIVE.RL_IDS[1], DRIVE.RL_OFFSETS, "RL"),
            new SwerveModule(3, DRIVE.RR_IDS[0], DRIVE.RR_IDS[1], DRIVE.RR_OFFSETS, "RR"));
        break;
      case SIM:
        SmartDashboard.putString("/Mode", "SIM");
        swerve = new Swerve(
            new GyroSim(),
            new SwerveModuleSim(0, DRIVE.FL_IDS[0], DRIVE.FL_IDS[1], DRIVE.FL_OFFSETS, "FL"),
            new SwerveModuleSim(1, DRIVE.FR_IDS[0], DRIVE.FR_IDS[1], DRIVE.FR_OFFSETS, "FR"),
            new SwerveModuleSim(2, DRIVE.RL_IDS[0], DRIVE.RL_IDS[1], DRIVE.RL_OFFSETS, "RL"),
            new SwerveModuleSim(3, DRIVE.RR_IDS[0], DRIVE.RR_IDS[1], DRIVE.RR_OFFSETS, "RR"));
        break;
      default:
      case SIM_REPLAY:
        SmartDashboard.putString("/Mode", "SIM_REPLAY");
        swerve = new Swerve(
            new GyroIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {});
        break;
    }

    swerve.setDefaultCommand(new TeleopDrive(swerve));
    
    vision = new Vision(swerve);
    autonChooser = new AutonChooser(swerve);
    // Configure the button bindings
    updateControllers();
  }


  private Command getCommandForPath(String pathName, boolean resetOdometry, PathConstraints constraints) {
    PathPlannerTrajectory path = PathPlanner.loadPath(pathName, constraints);
    if(path == null) {
      DriverStation.reportError("Failed to load path: " + pathName, true);
      return new InstantCommand(()->{
        DriverStation.reportError("Tried to execute path that failed to load! Path name: " + pathName, true);
      });
    }
    Command forwardCommand = swerve.followTrajectoryCommand(path, resetOdometry, true);
    return forwardCommand;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void updateControllers() {
    // Do nothing if controller layout hasn't changed.
    if (!Controllers.didControllersChange())
      return;
    System.out.println("Updating controller layout");

    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    // Put new bindings here.
    Controllers.driveController.getPPLoadDebugForwardPath()
    .toggleOnTrue(getCommandForPath("Start_To_Game_Piece_1", true, AUTON.SLOW_CONSTRAINTS));

    Controllers.driveController.getPPLoadDebugBackwardPath()
    .toggleOnTrue(getCommandForPath("Game_Piece_1_To_Start", true, AUTON.SLOW_CONSTRAINTS));

    // This command puts the robot 1 meter in front of apriltag 8 (middle of bottom left grid on pathplanner picture of 2023 field)
    Controllers.driveController.getPPGotoTag8().onTrue(new InstantCommand(()->{
      swerve.goTo(Constants.FIELD_CONSTANTS.DEBUG_GO_TO_DEST, AUTON.SLOW_CONSTRAINTS).schedule();
    }));

    Controllers.driveController.getResetPoseButton().onTrue(new InstantCommand(()->{
      swerve.setPose(new Pose2d(3,0.38,Rotation2d.fromDegrees(90)));
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelectedAuton();
  }
}
