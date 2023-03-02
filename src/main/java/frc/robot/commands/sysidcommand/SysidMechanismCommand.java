package frc.robot.commands.sysidcommand;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.robot.Robot;

// This is going to be slightly weird
public class SysidMechanismCommand extends CommandBase {
  private ArrayList<Double> data = new ArrayList<Double>();
  /* If the test is dynamic, false if quasistatic */
  private boolean isDynamicTest;

  /* Multiplied by time for quasistatic, voltage to set for dynamic */
  private double voltCommand;
  private boolean isWrongMech;

  private double startTime;

  /* Static so it persists between command runs */
  private static double ackNum = 0;

  private final int maxCapacity = 36000;

  private Runnable zeroEncoder;
  private Supplier<Double> getVoltage, getVelocity, getPosition;
  private Consumer<Double> setVoltage;


  // Runs as auton, tries to emulate the sysid interface. Note: may have potential issues due to other robot code running and taking up processing time
  public SysidMechanismCommand(Runnable zeroEncoder, Supplier<Double> getVoltage, Supplier<Double> getVelocity,
      Supplier<Double> getPosition, Consumer<Double> setVoltage, Subsystem... requirements) {

    addRequirements(requirements);

    this.zeroEncoder = zeroEncoder;
    this.getVelocity = getVelocity;
    this.getPosition = getPosition;
    this.setVoltage = setVoltage;
    this.getVoltage = getVoltage;

    // Realtime priorities
    if (Robot.isReal()) {
      // AFAIK this is already set by default
      Notifier.setHALThreadPriority(true, 40);
      // But this isn't
      Threads.setCurrentThreadPriority(true, 15);
    }

    data.ensureCapacity(maxCapacity);

    this.isDynamicTest = SmartDashboard.getString("SysIdTestType", "").toLowerCase().contains("dynamic");
    this.voltCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0);

    SmartDashboard.putNumber("SysIdAckNumber", ackNum);

    var test = SmartDashboard.getString("SysIdTest", "");
    this.isWrongMech = test.contains("Drivetrain");
    SmartDashboard.putBoolean("SysIdWrongMech", isWrongMech);

    this.startTime = Timer.getFPGATimestamp();

    this.zeroEncoder.run();
  }

  @Override
  public void execute() {
    if (isWrongMech) {
      System.out.println("Wrong mechanism!!");
      return;
    }

    // Set our motor speeds depending on which test is appropriate
    double targetVolts = 0;
    if (isDynamicTest) {
      targetVolts = voltCommand;
    } else {
      targetVolts = voltCommand * (Timer.getFPGATimestamp() - startTime);
    }
    setVoltage.accept(targetVolts);

    // Logging!
    data.add(Timer.getFPGATimestamp());
    // Note: due to our implementation this is probably behind by one periodic, and then maybe more if we're counting latency in other areas
    data.add(getVoltage.get());
    data.add(getPosition.get());
    data.add(getVelocity.get());
  }

  @Override
  public void end(boolean interrupted) {

    // This was probably false initially
    Threads.setCurrentThreadPriority(false, 0);
    SmartDashboard.putBoolean("SysIdOverflow", data.size() > maxCapacity);

    StringBuilder outputString = new StringBuilder();
    outputString.ensureCapacity(maxCapacity + 50);
    outputString.append(isDynamicTest ? "fast" : "slow");
    outputString.append(voltCommand > 0 ? "-forward;" : "-backward;");
    for (var entry : data) {
      outputString.append(entry);
      outputString.append(",");
    }
    outputString.deleteCharAt(outputString.length() - 1);

    SmartDashboard.putString("SysIdTelemetry", outputString.toString());
    SmartDashboard.putNumber("SysIdAckNumber", ++ackNum);
  }
}
