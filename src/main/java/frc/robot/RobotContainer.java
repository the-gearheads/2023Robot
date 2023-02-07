// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.sysidcommand.SysidCommand;
import frc.robot.controllers.Controllers;
import frc.robot.controllers.SingleXboxController;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.ShouldSetVisionPose;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should
 * be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final Vision vision;

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
        swerveSubsystem = new SwerveSubsystem(
            new SwerveModule(0, Drivetrain.FL_IDS[0], Drivetrain.FL_IDS[1], Drivetrain.FL_OFFSETS, "FL"),
            new SwerveModule(1, Drivetrain.FR_IDS[0], Drivetrain.FR_IDS[1], Drivetrain.FR_OFFSETS, "FR"),
            new SwerveModule(2, Drivetrain.RL_IDS[0], Drivetrain.RL_IDS[1], Drivetrain.RL_OFFSETS, "RL"),
            new SwerveModule(3, Drivetrain.RR_IDS[0], Drivetrain.RR_IDS[1], Drivetrain.RR_OFFSETS, "RR"));
        break;
      case SIM:
        SmartDashboard.putString("/Mode", "SIM");
        swerveSubsystem = new SwerveSubsystem(
            new SwerveModuleSim(0, Drivetrain.FL_IDS[0], Drivetrain.FL_IDS[1], Drivetrain.FL_OFFSETS, "FL"),
            new SwerveModuleSim(1, Drivetrain.FR_IDS[0], Drivetrain.FR_IDS[1], Drivetrain.FR_OFFSETS, "FR"),
            new SwerveModuleSim(2, Drivetrain.RL_IDS[0], Drivetrain.RL_IDS[1], Drivetrain.RL_OFFSETS, "RL"),
            new SwerveModuleSim(3, Drivetrain.RR_IDS[0], Drivetrain.RR_IDS[1], Drivetrain.RR_OFFSETS, "RR"));
        break;
      default:
      case SIM_REPLAY:
        SmartDashboard.putString("/Mode", "SIM_REPLAY");
        swerveSubsystem = new SwerveSubsystem(new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {},
            new SwerveModuleIO() {});
        break;
    }

    swerveSubsystem.setDefaultCommand(new TeleopDrive(swerveSubsystem));
    
    vision = new Vision(swerveSubsystem);
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
    Command forwardCommand = swerveSubsystem.followTrajectoryCommand(path, resetOdometry, true);
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
    XboxController controller = ((SingleXboxController) Controllers.activeController).controller;

    PathConstraints constraints = new PathConstraints(7, 3);

    PathPlannerTrajectory forwardTraj = PathPlanner.loadPath("DEBUG_Forward", constraints);
    Command forwardCommand = swerveSubsystem.followTrajectoryCommand(forwardTraj, true, true);
    new JoystickButton(controller, XboxController.Button.kY.value).toggleOnTrue(forwardCommand);

    Controllers.activeController.getPPLoadDebugForwardPath()
    .toggleOnTrue(getCommandForPath("Start_To_Game_Piece_1", true, constraints)
    .alongWith(new InstantCommand(()->{
      vision.setShouldSetVisionPose(ShouldSetVisionPose.UPDATE);
    })));

    Controllers.activeController.getPPLoadDebugBackwardPath()
    .toggleOnTrue(getCommandForPath("Game_Piece_1_To_Start", true, constraints)
    .alongWith(new InstantCommand(()->{
      vision.setShouldSetVisionPose(ShouldSetVisionPose.DONT_UPDATE);
    })));

    // Controllers.activeController.getPPLoadDebugForwardPath().toggleOnTrue(getCommandForPath("DEBUG_Forward", true));
    // Controllers.activeController.getPPLoadDebugBackwardPath().toggleOnTrue(getCommandForPath("DEBUG_Backward", true));
    Controllers.activeController.getPPLoadDebugLeftPath().toggleOnTrue(getCommandForPath("DEBUG_Left", true, constraints));
    Controllers.activeController.getPPLoadDebugRightPath().whileTrue(new RepeatCommand(new InstantCommand(()->{
      swerveSubsystem.setX();
    })));

    // This command puts the robot 1 meter in front of apriltag 8 (middle of bottom left grid on pathplanner picture of 2023 field)
    Controllers.activeController.getPPGotoTag8().onTrue(new InstantCommand(()->{
      swerveSubsystem.goTo(Constants.FieldConstants.GRID_8, constraints).schedule();
    }));

    Controllers.activeController.getResetPoseButton().onTrue(new InstantCommand(()->{
      swerveSubsystem.setPose(new Pose2d(3,0.38,Rotation2d.fromDegrees(90)));
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathConstraints constraints1 = new PathConstraints(7, 3);
    PathConstraints constraints3 = new PathConstraints(4, 2);
    PathConstraints constraints2 = new PathConstraints(2, 1);

    Command startToGamePiece = getCommandForPath("Start_To_Game_Piece_1", true, constraints1);
    Command gamePieceToStart = getCommandForPath("Game_Piece_1_To_Start", false, constraints3);
    Command startToChargingStation = getCommandForPath("Start_To_Charging_Station", false, constraints2);
    Command updateVision = new InstantCommand(()->{
      vision.setShouldSetVisionPose(ShouldSetVisionPose.UPDATE);
    });
    Command DONTupdateVision1 = new InstantCommand(()->{
      vision.setShouldSetVisionPose(ShouldSetVisionPose.DONT_UPDATE);
    });
    Command DONTupdateVision2 = new InstantCommand(()->{
      vision.setShouldSetVisionPose(ShouldSetVisionPose.DONT_UPDATE);
    });
    return new ParallelRaceGroup(startToGamePiece, updateVision)
    .andThen(new ParallelRaceGroup(gamePieceToStart))
    .andThen(new ParallelRaceGroup(startToChargingStation))
    .andThen(new InstantCommand(()->{
      swerveSubsystem.setX();
    }));
  }
}
