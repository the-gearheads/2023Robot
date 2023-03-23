// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DRIVE;
import frc.robot.commands.arm.JoystickArmControl;
import frc.robot.commands.arm.SetArmPose;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.arm.Throw;
import frc.robot.commands.arm.ThrowState;
import frc.robot.commands.arm.SetArmPose.ArmPose;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.drive.autoalign.AutoAlign;
import frc.robot.commands.vision.FuseVisionEstimate;
import frc.robot.commands.wrist.AltWristControl;
import frc.robot.commands.wrist.ManualWristControl;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.auton.AutonChooser;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristSim;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.util.MoreMath;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleSim;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should
 * be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve swerve;
  @SuppressWarnings("unused")
  private Vision vision;
  private final AutonChooser autonChooser;
  private final Arm arm;
  private Wrist wrist;
  private Grabber grabber;
  private Leds leds;

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
        swerve = new Swerve(new Gyro(SPI.Port.kMXP, true),
            new SwerveModule(0, DRIVE.FL_IDS[0], DRIVE.FL_IDS[1], DRIVE.FL_OFFSETS, "FL"),
            new SwerveModule(1, DRIVE.FR_IDS[0], DRIVE.FR_IDS[1], DRIVE.FR_OFFSETS, "FR"),
            new SwerveModule(2, DRIVE.RL_IDS[0], DRIVE.RL_IDS[1], DRIVE.RL_OFFSETS, "RL"),
            new SwerveModule(3, DRIVE.RR_IDS[0], DRIVE.RR_IDS[1], DRIVE.RR_OFFSETS, "RR"));
        arm = new Arm();
        wrist = new Wrist(arm);
        vision = new Vision(swerve);
        break;
      case SIM:
        SmartDashboard.putString("/Mode", "SIM");
        swerve =
            new Swerve(new GyroSim(), new SwerveModuleSim(0, DRIVE.FL_IDS[0], DRIVE.FL_IDS[1], DRIVE.FL_OFFSETS, "FL"),
                new SwerveModuleSim(1, DRIVE.FR_IDS[0], DRIVE.FR_IDS[1], DRIVE.FR_OFFSETS, "FR"),
                new SwerveModuleSim(2, DRIVE.RL_IDS[0], DRIVE.RL_IDS[1], DRIVE.RL_OFFSETS, "RL"),
                new SwerveModuleSim(3, DRIVE.RR_IDS[0], DRIVE.RR_IDS[1], DRIVE.RR_OFFSETS, "RR"));
        arm = new ArmSim();
        wrist = new WristSim((ArmSim) arm);
        vision = new Vision(swerve);
        // vision.setDefaultCommand(new FuseVisionEstimate(vision, ConfidenceStrat.MECH_ADV));
        break;
      default:
      case SIM_REPLAY:
        SmartDashboard.putString("/Mode", "SIM_REPLAY");
        swerve = new Swerve(new GyroIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {},
            new SwerveModuleIO() {});
        arm = new ArmSim();
        wrist = new WristSim((ArmSim) arm);
        vision = new Vision(swerve);
        break;
    }

    grabber = new Grabber();
    autonChooser = new AutonChooser(swerve, arm, wrist, grabber);
    leds = new Leds();
    // Configure the button binding

    // swerve.setDefaultCommand(new TeleopDrive(swerve));
    arm.setDefaultCommand(new JoystickArmControl(arm));
    vision.setDefaultCommand(new FuseVisionEstimate(vision).ignoringDisable(true));
    updateControllers();

    // PortForwarder.add(5800, "photonvision.local", 5800);
    // PortForwarder.add(5801, "photonvisionpi.local", 5800);
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
    // Controllers.driverController.getPPLoadDebugForwardPath()
    //     .toggleOnTrue(AutonPaths.getCommandForPath("StartN1-GamePiece1", true, AUTON.SLOW_CONSTRAINTS, swerve));

    // Controllers.driverController.getPPLoadDebugBackwardPath()
    //     .toggleOnTrue(AutonPaths.getCommandForPath("GamePiece1-StartN1", true, AUTON.SLOW_CONSTRAINTS, swerve));

    // This command puts the robot 1 meter in front of apriltag 8 (middle of bottom left grid on pathplanner picture of 2023 field)
    // Controllers.driverController.getPPGotoTag8().onTrue(new InstantCommand(() -> {
    //   swerve.goTo(Constants.FIELD_CONSTANTS.DEBUG_GO_TO_DEST, AUTON.SLOW_CONSTRAINTS).schedule();
    // }));

    // Controllers.driverController.getResetPoseButton().onTrue(new InstantCommand(() -> {
    //   swerve.setPose(new Pose2d(3, 0.38, Rotation2d.fromDegrees(90)));
    // }));

    Controllers.driverController.backUpFromFeeder().onTrue(new ProxyCommand(() -> {
      var currentPose = swerve.getPose();
      var dest = MoreMath.deepCopyPose(currentPose);
      var translation = new Translation2d(Units.inchesToMeters(-4), 0);
      dest = new Pose2d(dest.getTranslation().plus(translation), dest.getRotation());
      return swerve.goTo(dest, Constants.AUTON.MID_CONSTRAINTS);
    }));
    // Controllers.driverController.getResetPoseButton().onTrue(new InstantCommand(() -> {
    //   swerve.setPose(new Pose2d());
    // }, swerve));
    Controllers.driverController.getAutoAlign().onTrue(new AutoAlign(swerve, arm));

    Controllers.operatorController.armGoToLowNode()
        .onTrue(new SetArmPose(arm, ArmPose.LOW_NODE).andThen(new ManualWristControl(wrist, WristState.RIGHT)));
    Controllers.operatorController.armGoToMidNode().onTrue(new SetArmPose(arm, ArmPose.MID_NODE));
    Controllers.operatorController.armGoToHighNode().onTrue(new SetArmPose(arm, ArmPose.HIGH_NODE));
    Controllers.operatorController.armGoToInsideRobotNode().onTrue(new StowArm(arm, wrist));
    Controllers.operatorController.armGoToFeederStationNode().onTrue(new SetArmPose(arm, ArmPose.FEEDER_STATION));
    Controllers.operatorController.setWristAlternatePose().whileTrue(new AltWristControl(wrist).repeatedly());
    Controllers.operatorController.openGrabber().whileTrue(new StartEndCommand(grabber::open, grabber::close, grabber));
    Controllers.operatorController.setArmByJoystick().onTrue(new JoystickArmControl(arm));

    Controllers.operatorController.throwCube()
        .onTrue(new Throw(arm, wrist, grabber, leds, new ThrowState(-45, 80, 20)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelectedAuton();
  }

  public void setTeleopDefault() {
    swerve.setDefaultCommand(new TeleopDrive(swerve));
  }

  public void clearTeleopDefault() {
    swerve.setDefaultCommand(new InstantCommand(() -> {
    }, swerve));
  }
}
