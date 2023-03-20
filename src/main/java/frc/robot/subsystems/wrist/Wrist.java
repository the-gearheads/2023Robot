// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WRIST;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.WristState.WristControlType;
import frc.robot.util.MoreMath;
import frc.robot.util.SendableArmFeedforward;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private int zeroCount = 0;
  private Boolean configureHasRan = false;

  private double goal;
  private WristControlType controlState = WristControlType.DEFAULT;

  private CANSparkMax motor = new CANSparkMax(WRIST.WRIST_ID, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

  protected PIDController pid = new PIDController(WRIST.WRIST_PID[0], WRIST.WRIST_PID[1], WRIST.WRIST_PID[2]);
  private SendableArmFeedforward ff =
      new SendableArmFeedforward(WRIST.WRIST_FF[0], WRIST.WRIST_FF[1], WRIST.WRIST_FF[2]);

  private double pidval;
  private double ffval;

  private double lastNonWrapRangePose = 90;

  private Arm arm;
  private boolean wrapRangeEntered = false;
  private double volts;

  public Wrist(Arm arm) {
    this.arm = arm;
    configure();

    SmartDashboard.putData("Wrist/ff", ff);
    SmartDashboard.putData("Wrist/pid", pid);
  }

  private void setControlState(WristControlType controlState) {
    this.controlState = controlState;
  }

  public double getGoal() {
    return goal;
  }

  public double getPose() {
    return encoder.getPosition() + Constants.WRIST.ANGLE_OFFSET;
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  //In case a wrist command needs to access arm pose (don't want to give it entire arm subsystem)
  public double getArmPose() {
    return arm.getPose();
  }

  public void setVoltage(double volts) {
    this.volts = volts;
    motor.setVoltage(volts);
  }

  public void setGoal(WristState state) {
    var desiredGoal = state.getGoal();

    if (angersWrapRangeHandler(desiredGoal) || angersInsideRobotHandler(desiredGoal)){
      setGoalByType(WristControlType.DEFAULT);
    }else{
      goal = desiredGoal;
      setControlState(state.type);
    }
  }

  private boolean angersInsideRobotHandler(double goal) {
    var armInsideBot = WristState.INSIDE_ROBOT.inRange(getArmPose());
    return armInsideBot && goal < 0;
  }

  private boolean angersWrapRangeHandler(double goal) {
    return goal > WRIST.WRAP_RANGE_UPPER_BOUND || goal < WRIST.WRAP_RANGE_LOWER_BOUND;
  }

  @Override
  public void periodic() {
    runPid();
    wrapRangeHandler();
    insideRobotHandler();
    sensorFaultHandler();
    log();
  }

  private void runPid() {
    double currentPose = getPose();
    this.pidval = pid.calculate(currentPose, goal);
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
      Logger.getInstance().recordOutput("Wrist/wrap range handler triggered", true);
    } else{
      lastNonWrapRangePose = getPose();
      Logger.getInstance().recordOutput("Wrist/wrap range handler triggered", false);
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
    Logger.getInstance().recordOutput("Wrist/Inside Robot Handler Triggered", overrideTriggered);
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
    Logger.getInstance().recordOutput("Wrist/Faults/Zero Count Fault", zeroCountFault);
    Logger.getInstance().recordOutput("Wrist/Faults/Fault", hasFaults);
    Logger.getInstance().recordOutput("Wrist/Faults/Sticky Fault", hasStickyFaults);

    if (hasStickyFaults) {
      DriverStation.reportWarning("Wrist Sticky Fault", true);
    }

    var shouldPanic = zeroCountFault || hasFaults;
    if (shouldPanic) {
      DriverStation.reportError("OUR ZERO ERROR IN WRIST", true);
      setVoltage(0);
      if (configureHasRan == false) {
        configure();
      }
      configureHasRan = true;
    }
  }

  private void log() {
    Logger.getInstance().recordOutput("Wrist/wrap range entered", wrapRangeEntered);
    Logger.getInstance().recordOutput("Wrist/Last non wrap pose", lastNonWrapRangePose);
    Logger.getInstance().recordOutput("Wrist/ReConfigure has ran", configureHasRan);
    Logger.getInstance().recordOutput("Wrist/control state", controlState.name());
    Logger.getInstance().recordOutput("Wrist/Pose", MoreMath.round(getPose(), 1));
    Logger.getInstance().recordOutput("Wrist/Vel", MoreMath.round(getVelocity(), 1));
    Logger.getInstance().recordOutput("Wrist/Goal", MoreMath.round(goal, 1));
    Logger.getInstance().recordOutput("Wrist/Error", MoreMath.round(pid.getPositionError(), 1));
    Logger.getInstance().recordOutput("Wrist/PIDVal", MoreMath.round(pidval, 1));
    Logger.getInstance().recordOutput("Wrist/FFVal", MoreMath.round(ffval, 1));
    Logger.getInstance().recordOutput("Wrist/Appliedvolts", MoreMath.round(motor.getAppliedOutput(), 1));
    Logger.getInstance().recordOutput("Wrist/Current Command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "");
  }

  private void configure() {
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    encoder.setInverted(true);
    motor.setIdleMode(IdleMode.kCoast);
    encoder.setPositionConversionFactor(360);
    encoder.setVelocityConversionFactor(360);

    /* Status 0 governs applied output, faults, and whether is a follower. Not important for this. */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    /* Integrated motor position isn't important here. */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    /* Don't have an analog sensor */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
    /* Don't have an alternate encoder */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    /* Have a duty cycle encoder */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    try {
      Thread.sleep((long) 40.0);
    } catch (Exception e) {
      e.printStackTrace();
    } ;
  }
}
