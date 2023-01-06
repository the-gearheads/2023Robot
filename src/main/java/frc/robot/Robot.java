// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.RobotMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  @SuppressWarnings("all")
  public void robotInit() {

    setUseTiming(Constants.getMode() != RobotMode.SIM_REPLAY); // Run as fast as possible during replay
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
    Logger.getInstance().recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.getInstance().recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.getInstance().recordMetadata("IsDirty", Boolean.toString(BuildConstants.DIRTY != 0));
    
    if (Constants.getMode() == RobotMode.REAL) {
      Logger.getInstance().addDataReceiver(new ByteLogReceiver("/home/lvuser")); // Log to home directory
      Logger.getInstance().addDataReceiver(new LogSocketServer(5800)); // Provide log data over the network, viewable in Advantage Scope.
    } else if(Constants.getMode() == RobotMode.SIM_REPLAY) {
      String path = ByteLogReplay.promptForPath(); // Prompt the user for a file path on the command line
      Logger.getInstance().setReplaySource(new ByteLogReplay(path)); // Read log file for replay
      Logger.getInstance().addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim"))); // Save replay results to a new log with the "_sim" suffix
    } else { // Probably sim (non-replay)
      Logger.getInstance().addDataReceiver(new ByteLogReceiver("./")); // Log to current directory
      Logger.getInstance().addDataReceiver(new LogSocketServer(5800)); // Provide log data over the network, viewable in Advantage Scope.
    }
    
    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    PathPlannerServer.startServer(5811);
    LiveWindow.disableAllTelemetry();
    Constants.processAnnotations(Constants.class);
    /* Somewhat cursed but it'll work */
    SmartDashboard.putData("Constants", new Constants());
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.updateControllers();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
