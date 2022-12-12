// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.TeleopDrive;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleSim;
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default teleop command
    switch(Constants.getMode()) {
      case SIM:
      swerveSubsystem = new SwerveSubsystem(
        new SwerveModuleSim(Drivetrain.FL_DRIVE_ID, Drivetrain.FL_STEER_ID, Drivetrain.FL_OFFSET, "FL", true),
        new SwerveModuleSim(Drivetrain.FR_DRIVE_ID, Drivetrain.FR_STEER_ID, Drivetrain.FR_OFFSET, "FR", true),
        new SwerveModuleSim(Drivetrain.RL_DRIVE_ID, Drivetrain.RL_STEER_ID, Drivetrain.RL_OFFSET, "RL", false),
        new SwerveModuleSim(Drivetrain.RR_DRIVE_ID, Drivetrain.RR_STEER_ID, Drivetrain.RR_OFFSET, "RR", false)
      );
      break;

      case SIM_REPLAY:
        swerveSubsystem = new SwerveSubsystem(new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {});
        break;

      default:
      case REAL:
      swerveSubsystem = new SwerveSubsystem(
        new SwerveModule(Drivetrain.FL_DRIVE_ID, Drivetrain.FL_STEER_ID, Drivetrain.FL_OFFSET, "FL", true),
        new SwerveModule(Drivetrain.FR_DRIVE_ID, Drivetrain.FR_STEER_ID, Drivetrain.FR_OFFSET, "FR", true),
        new SwerveModule(Drivetrain.RL_DRIVE_ID, Drivetrain.RL_STEER_ID, Drivetrain.RL_OFFSET, "RL", false),
        new SwerveModule(Drivetrain.RR_DRIVE_ID, Drivetrain.RR_STEER_ID, Drivetrain.RR_OFFSET, "RR", false)
      );
      break;
    }

    swerveSubsystem.setDefaultCommand(new TeleopDrive(swerveSubsystem));
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
    CommandScheduler.getInstance().clearButtons();

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
