// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.RobotMode;
import frc.robot.auton.TestPlaceThenDock;
import frc.robot.subsystems.leds.LedState;
import frc.robot.util.NTToAdvantageKit;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation. If you change
 * the name of this class or the package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
  private Command autonCommand;
  private RobotContainer robotContainer;

  /* Globals :( */
  public static double matchTime = -1;
  private static double matchTimeStart = 0;

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  @SuppressWarnings("all")
  public void robotInit() {

    setUseTiming(Constants.getMode() != RobotMode.SIM_REPLAY); // Run as fast as possible during replay
    Logger.getInstance().recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.getInstance().recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.getInstance().recordMetadata("IsDirty", Boolean.toString(BuildConstants.DIRTY != 0));

    if (Constants.getMode() == RobotMode.REAL) {
      Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else if (Constants.getMode() == RobotMode.SIM_REPLAY) {
      String path = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.getInstance().setReplaySource(new WPILOGReader(path)); // Read replay log
      Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim"))); // Save outputs to a new log
    } else { // Probably sim (non-replay)
      Logger.getInstance().addDataReceiver(new WPILOGWriter("./")); // Log to current directory
      Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    PathPlannerServer.startServer(5811);
    LiveWindow.disableAllTelemetry();
    Constants.processAnnotations(Constants.class);
    /* Somewhat cursed but it'll work */
    SmartDashboard.putData("Constants", new Constants());
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    NTToAdvantageKit.periodic();
    robotContainer.updateControllers();

    if (DriverStation.isEStopped()) {
      robotContainer.getLeds().setState(LedState.HOT_PINK);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.setDisabledVisionStrat();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    matchTime = -1;
    autonCommand = robotContainer.getAutonomousCommand();
    robotContainer.clearTeleopDefault();

    // schedule the autonomous command (example)
    if (autonCommand != null) {
      autonCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonCommand != null) {
      autonCommand.cancel();
    }
    TestPlaceThenDock.initDockTestingTelemetry();
    robotContainer.setTeleopDefault();
    CANSparkMaxLowLevel.enableExternalUSBControl(false);
    robotContainer.setTeleopVisionStrat();
    matchTimeStart = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    matchTime = 135 - (Timer.getFPGATimestamp() - matchTimeStart);
    matchTime = matchTime < 0 ? 0 : matchTime;
    Logger.getInstance().recordOutput("TeleopMatchTime", matchTime);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CANSparkMaxLowLevel.enableExternalUSBControl(true);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
