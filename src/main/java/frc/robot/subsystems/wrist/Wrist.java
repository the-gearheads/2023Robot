// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WRIST;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.WristState.WristControlType;
import frc.robot.util.MoreMath;
import frc.robot.util.RevConfigUtils;
import frc.robot.util.SendableArmFeedforward;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax motor = new CANSparkMax(WRIST.WRIST_ID, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

  protected PIDController pid = new PIDController(WRIST.WRIST_PID[0], WRIST.WRIST_PID[1], WRIST.WRIST_PID[2]);
  private SendableArmFeedforward ff =
      new SendableArmFeedforward(WRIST.WRIST_FF[0], WRIST.WRIST_FF[1], WRIST.WRIST_FF[2]);

  private double pidval;
  private double ffval;
  private double volts;

  private WristState goal = WristState.INSIDE_ROBOT;

  private double lastNonWrapRangePose = 90;
  private boolean wrapRangeEntered = false;

  private int zeroCount = 0;
  private Boolean configureHasRan = false;

  private boolean shouldFallBackToDefault = true;

  private Arm arm;

  public Wrist(Arm arm) {
    this.arm = arm;
    RevConfigUtils.configure(this::configure, "Wrist");

    SmartDashboard.putData("Wrist/ff", ff);
    SmartDashboard.putData("Wrist/pid", pid);
  }

  public WristState getGoal() {
    return goal;
  }

  public double getPose() {
    return -encoder.getPosition() + Constants.WRIST.ANGLE_OFFSET;
  }

  public double getVelocity() {
    return -encoder.getVelocity();
  }

  //In case a wrist command needs to access arm pose (don't want to give it entire arm subsystem)
  public double getArmPose() {
    return arm.getPose();
  }

  public void setVoltage(double volts) {
    this.volts = volts;
    motor.setVoltage(-volts);
  }

  public void setGoal(WristState state) {
    if (angersWrapRangeHandler(state.getGoal()) || angersInsideRobotHandler(state.getGoal()))
      return;
    goal = state;

    if (state.type != WristControlType.DEFAULT)
      shouldFallBackToDefault = false;
  }

  private boolean angersInsideRobotHandler(double goal) {
    var armInsideBot = WristState.INSIDE_ROBOT.inRange(getArmPose());
    var angered = armInsideBot && goal < 0;

    Logger.getInstance().recordOutput("Wrist/inside robot handler/proactively triggered", angered);
    return angered;
  }

  private boolean angersWrapRangeHandler(double goal) {
    var angered = goal > WRIST.WRAP_RANGE_UPPER_BOUND || goal < WRIST.WRAP_RANGE_LOWER_BOUND;
    Logger.getInstance().recordOutput("Wrist/wrap range handler/proactively triggered", angered);
    return angered;
  }

  @Override
  public void periodic() {
    initDefaultModeHander();
    runPid();
    wrapRangeHandler();
    insideRobotHandler();
    sensorFaultHandler();
    log();
    finalDefaultModeHander();
  }

  private void initDefaultModeHander() {
    if (shouldFallBackToDefault) {
      setGoalByType(WristControlType.DEFAULT);
    }
  }

  private void finalDefaultModeHander() {
    shouldFallBackToDefault = true;
  }

  private void runPid() {
    double currentPose = getPose();
    this.pidval = pid.calculate(currentPose, goal.getGoal());
    this.ffval = ff.calculate(currentPose, 0);

    setVoltage(ffval + pidval);
  }

  private void wrapRangeHandler() {
    var inWrapRange = getPose() > WRIST.WRAP_RANGE_UPPER_BOUND || getPose() < WRIST.WRAP_RANGE_LOWER_BOUND;
    if (inWrapRange)
      wrapRangeEntered = true;
    else {
      var didLoopBack = Math.signum(lastNonWrapRangePose) == Math.signum(getPose());
      if (didLoopBack)
        wrapRangeEntered = false;
    }

    if (wrapRangeEntered) {
      var direction = -1 * Math.signum(lastNonWrapRangePose);
      setVoltage(direction * WRIST.WRAP_RANGE_SPEED);
      Logger.getInstance().recordOutput("Wrist/wrap range handler/triggered", true);
    } else {
      lastNonWrapRangePose = getPose();
      Logger.getInstance().recordOutput("Wrist/wrap range handler/triggered", false);
    }

  }

  /* Don't move towards the base of the robot if inside it (not good) */
  private void insideRobotHandler() {
    var armInsideBot = WristState.INSIDE_ROBOT.inRange(getArmPose());
    var inQuad3 = MoreMath.within(getPose(), -180, -90);
    var inQuad4 = MoreMath.within(getPose(), -90, 0);
    var wristMovingDown = (inQuad3 && volts > 0) || (inQuad4 && volts < 0);

    var overrideTriggered = false;
    if (armInsideBot && wristMovingDown) {
      setVoltage(0);
      overrideTriggered = true;
    }
    Logger.getInstance().recordOutput("Wrist/inside robot handler/triggered", overrideTriggered);
  }

  public void setGoalByType(WristControlType wristStateType) { // check what range the arm is in and set the wrist accordingly
    double armPos = arm.getPose();
    for (WristState wristState : WristState.values()) {
      if (wristState.type == wristStateType && wristState.inRange(armPos)) {
        setGoal(wristState);
        return;
      }
    }
  }

  public void sensorFaultHandler() {
    boolean hasFaults = motor.getFault(FaultID.kCANTX) || motor.getFault(FaultID.kCANRX);
    boolean hasStickyFaults = motor.getStickyFault(FaultID.kCANTX) || motor.getStickyFault(FaultID.kCANRX);
    var pose = encoder.getPosition();

    if (pose == 0 || pose > 2000 || pose < -2000) {
      zeroCount++;
    }

    var zeroCountFault = zeroCount > 1;
    Logger.getInstance().recordOutput("Wrist/fault handler/zero fault count", zeroCountFault);
    Logger.getInstance().recordOutput("Wrist/fault handler/fault", hasFaults);
    Logger.getInstance().recordOutput("Wrist/fault handler/sticky fault", hasStickyFaults);

    if (hasStickyFaults) {
      DriverStation.reportWarning("Wrist Sticky Fault", true);
    }

    var shouldPanic = zeroCountFault || hasFaults;
    if (shouldPanic) {
      DriverStation.reportError("OUR ZERO ERROR IN WRIST", true);
      setVoltage(0);
      // if (configureHasRan == false) {
      if (Math.floor(Timer.getFPGATimestamp()) % 2 == 0) {
        RevConfigUtils.configure(this::configure, "Wrist");
      }
      // }
      configureHasRan = true;
    }
  }

  private void log() {
    Logger.getInstance().recordOutput("Wrist/wrap range handler/range entered", wrapRangeEntered);
    Logger.getInstance().recordOutput("Wrist/wrap range handler/Last non wrap pose", lastNonWrapRangePose);
    Logger.getInstance().recordOutput("Wrist/fault handler/ReConfigure has ran", configureHasRan);
    Logger.getInstance().recordOutput("Wrist/control state", goal.type.name());
    Logger.getInstance().recordOutput("Wrist/Pose", MoreMath.round(getPose(), 1));
    Logger.getInstance().recordOutput("Wrist/Vel", MoreMath.round(getVelocity(), 1));
    Logger.getInstance().recordOutput("Wrist/Goal", MoreMath.round(goal.getGoal(), 1));
    Logger.getInstance().recordOutput("Wrist/Error", MoreMath.round(pid.getPositionError(), 1));
    Logger.getInstance().recordOutput("Wrist/PIDVal", MoreMath.round(pidval, 1));
    Logger.getInstance().recordOutput("Wrist/FFVal", MoreMath.round(ffval, 1));
    Logger.getInstance().recordOutput("Wrist/Appliedvolts", MoreMath.round(motor.getAppliedOutput(), 3));
    Logger.getInstance().recordOutput("Wrist/Current Command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "");
    Logger.getInstance().recordOutput("Wrist/default fallback handler/fell back to default", shouldFallBackToDefault);

  }

  public ArrayList<REVLibError> configure() {
    ArrayList<REVLibError> e = new ArrayList<>();

    e.add(motor.restoreFactoryDefaults());
    motor.setInverted(false);
    e.add(encoder.setInverted(false));
    e.add(motor.setIdleMode(IdleMode.kCoast));
    e.add(encoder.setPositionConversionFactor(360));
    e.add(encoder.setVelocityConversionFactor(360));

    /* Status 0 governs applied output, faults, and whether is a follower. Not important for this. */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20));
    /* Integrated motor position isn't important here. */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500));
    /* Don't have an analog sensor */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500));
    /* Don't have an alternate encoder */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500));
    /* Have a duty cycle encoder */
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20));
    e.add(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20));

    return e;
  }
}
