package frc.robot.commands.sysidcommand;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.ArrayList;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveSubsystem;

// This is going to be slightly weird
public class SysidCommand extends CommandBase {
  private SwerveSubsystem drive;
  private ArrayList<Double> data = new ArrayList<Double>();
  /* If the test is dynamic, false if quasistatic */
  private boolean isDynamicTest;
  /* Whether to rotate or not when doing the test */
  private boolean rotateInTest;
  /* Multiplied by time for quasistatic, voltage to set for dynamic */
  private double voltCommand;
  private boolean isWrongMech;

  private double startTime;

  /* Static so it persists between command runs */
  private static double ackNum = 0;

  private final int maxCapacity = 36000;

  // Runs as auton, tries to emulate the sysid interface. Note: may have potential issues due to other robot code running and taking up processing time
  public SysidCommand(SwerveSubsystem drive) {
    addRequirements(drive);
    this.drive = drive;

    // Realtime priorities
    if(Robot.isReal()) {
      // AFAIK this is already set by default
      Notifier.setHALThreadPriority(true, 40);
      // But this isn't
      Threads.setCurrentThreadPriority(true, 15);
    }

    data.ensureCapacity(maxCapacity);

    this.isDynamicTest = SmartDashboard.getString("SysIdTestType", "").toLowerCase().contains("dynamic");
    this.rotateInTest = SmartDashboard.getBoolean("SysIdRotate", false);
    this.voltCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0);
    this.isWrongMech = !SmartDashboard.getString("SysIdTest", "").contains("Drivetrain");
    SmartDashboard.putBoolean("SysIdWrongMech", isWrongMech);
    SmartDashboard.putNumber("SysIdAckNumber", ackNum);
  
    this.startTime = Timer.getFPGATimestamp();

    drive.zeroEncoders();
  }

  @Override
  public void execute() {
    //if(isWrongMech) {System.out.println("Wrong mechanism!!"); return;}

    // Set our motor speeds depending on which test is appropriate
    double targetVolts = 0;
    if(isDynamicTest) {
      targetVolts = voltCommand;
    } else {
      targetVolts = voltCommand * (Timer.getFPGATimestamp() - startTime);
    }
    drive.setVolts(targetVolts * (rotateInTest ? -1 : 1), targetVolts);


    var voltages = drive.getDiffVoltages();
    var velocities = drive.getDiffVelocities();
    var positions = drive.getDiffPositions();

    // Logging!
    data.add(Timer.getFPGATimestamp());
    // Note: due to our implementation this is probably behind by one periodic, and then maybe more if we're counting latency in other areas
    data.add(voltages[0]);
    data.add(voltages[1]);

    data.add(positions[0]);
    data.add(positions[1]);

    data.add(velocities[0]);
    data.add(velocities[1]);
    data.add(drive.getContinuousGyroAngleRad());
    data.add(drive.getAngularVelRad());

    SmartDashboard.putBoolean("is being dynmaic", isDynamicTest);
  }

  @Override
  public void end(boolean interrupted) {

    // This was probably false initially
    Threads.setCurrentThreadPriority(false, 0);
    SmartDashboard.putBoolean("SysIdOverflow", data.size() > maxCapacity);

    StringBuilder outputString = new StringBuilder();
    outputString.ensureCapacity(maxCapacity + 50);
    outputString.append(isDynamicTest ? "fast" : "slow");
    outputString.append(voltCommand > 0 ? "-forward;": "-backward;");
    for(var entry: data) {
      outputString.append(entry);
      outputString.append(",");
    }
    outputString.deleteCharAt(outputString.length()-1);

    SmartDashboard.putString("SysIdTelemetry", outputString.toString());
    SmartDashboard.putNumber("SysIdAckNumber", ++ackNum);
  }

  
}
