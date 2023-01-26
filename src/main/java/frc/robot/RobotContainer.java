// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.vision.TrackAprilTags;
import frc.robot.commands.vision.UpdatePoseEstimator;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final Vision vision = new Vision();

  public String readPipelineFile() {
    try {
      return Files.readString(Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "streampath.json"), Charset.forName("UTF-8"));
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
    switch(Constants.getMode()) {
      case REAL:
        SmartDashboard.putString("/Mode", "REAL");
        swerveSubsystem = new SwerveSubsystem(
          new SwerveModule(0, Drivetrain.FL_IDS[0], Drivetrain.FL_IDS[1], Drivetrain.FL_OFFSETS, "FL"),
          new SwerveModule(1, Drivetrain.FR_IDS[0], Drivetrain.FR_IDS[1], Drivetrain.FR_OFFSETS, "FR"),
          new SwerveModule(2, Drivetrain.RL_IDS[0], Drivetrain.RL_IDS[1], Drivetrain.RL_OFFSETS, "RL"),
          new SwerveModule(3, Drivetrain.RR_IDS[0], Drivetrain.RR_IDS[1], Drivetrain.RR_OFFSETS, "RR")
        );
        break;
        default:
        case SIM_REPLAY:
          SmartDashboard.putString("/Mode", "SIM_REPLAY");
          swerveSubsystem = new SwerveSubsystem(new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {});
        break;
    }

    swerveSubsystem.setDefaultCommand(new TeleopDrive(swerveSubsystem));
    vision.setDefaultCommand(new UpdatePoseEstimator(vision, swerveSubsystem));
    // Configure the button bindings
    updateControllers();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void updateControllers() {
    // Do nothing if controller layout hasn't changed.
    if(!Controllers.didControllersChange()) return; 
    System.out.println("Updating controller layout");

    // Clear buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Find new controllers
    Controllers.updateActiveControllerInstance();

    // Put new bindings here. 
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>(){{
      add(PathPlanner.loadPath("Forward2Meters", Drivetrain.Auton.CONSTRAINTS));
      add(PathPlanner.loadPath("Right2Meters", Drivetrain.Auton.CONSTRAINTS));
      add(PathPlanner.loadPath("DiagonalDown2Left2", Drivetrain.Auton.CONSTRAINTS));
    }};
    return new RepeatCommand(new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
          swerveSubsystem.setPose(trajectories.get(0).getInitialHolonomicPose());
      }).andThen(swerveSubsystem.followTrajectoriesCommand(trajectories, true)));
  }
}
