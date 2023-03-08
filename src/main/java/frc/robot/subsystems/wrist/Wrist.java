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

  protected PIDController pid = 
      new PIDController(WRIST.WRIST_PID[0], WRIST.WRIST_PID[1], WRIST.WRIST_PID[2]);
  private SendableArmFeedforward ff =
      new SendableArmFeedforward(WRIST.WRIST_FF[0], WRIST.WRIST_FF[1], WRIST.WRIST_FF[2]);
  
  private double pidval;
  private double ffval;

  private double lastValidPose=90;
  
  private Arm arm;
  private boolean invalidReached=false;

  public Wrist(Arm arm) {
    this.arm = arm;
    configure();

    SmartDashboard.putData("Wrist/ff", ff);
    SmartDashboard.putData("Wrist/pid", pid);
  }

  private void setControlState(WristControlType controlState){
    this.controlState = controlState;
  }

  public double getGoal() {
    return goal;
  }

  public void setGoal(WristState state) {
    goal = state.getWristGoal(arm.getPose());
    setControlState(state.type);
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
    motor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("reached here", false);
    if(controlState==WristControlType.DEFAULT){
      setGoalByType(WristControlType.DEFAULT);
    }

    double currentPose = getPose();
    this.pidval = pid.calculate(currentPose, goal);
    this.ffval = ff.calculate(currentPose, 0);

    setVoltage(ffval + pidval);

    if(inInvalidRange()){
      invalidReached=true;
      var goal = WristState.VARIABLE;
      goal.setWristGoal(lastValidPose);
      setGoal(goal);
    }else{
      if(invalidReached==false){
        lastValidPose = currentPose;
      }
    }

    if(invalidReached){
      if(!inInvalidRange() && (Math.signum(lastValidPose) == Math.signum(currentPose))){
        invalidReached=false;
      }else{
        if(lastValidPose>0){
          setVoltage(-2);
        }else{
          setVoltage(2);
        }
      }
    }

    if(sensorErrorHandler()){
      DriverStation.reportError("OUR ZERO ERROR IN WRIST", null);
      setVoltage(0);
      if (configureHasRan == false) {
        configure();
      }
      configureHasRan = true;
    }
    log();
    setControlState(WristControlType.DEFAULT);
  }

  private boolean inInvalidRange() {
    var currentPose = getPose();
    return currentPose>135 || currentPose<-135;
  }

  private void log(){
    Logger.getInstance().recordOutput("Wrist/invalid reached", invalidReached);
    Logger.getInstance().recordOutput("Wrist/Last Valid Pose", lastValidPose);
    Logger.getInstance().recordOutput("Wrist/ReConfigure has ran", configureHasRan);
    Logger.getInstance().recordOutput("Wrist/control state", controlState.name());
    Logger.getInstance().recordOutput("Wrist/Pose", MoreMath.round(getPose(),1));
    Logger.getInstance().recordOutput("Wrist/Vel", MoreMath.round(getVelocity(),1));
    Logger.getInstance().recordOutput("Wrist/Goal", MoreMath.round(goal,1));
    Logger.getInstance().recordOutput("Wrist/Error", MoreMath.round(pid.getPositionError(),1));
    Logger.getInstance().recordOutput("Wrist/PIDVal", MoreMath.round(pidval,1));
    Logger.getInstance().recordOutput("Wrist/FFVal", MoreMath.round(ffval,1));
    Logger.getInstance().recordOutput("Wrist/Appliedvolts", MoreMath.round(motor.getAppliedOutput(), 1));
    Logger.getInstance().recordOutput("Wrist/Current Command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "");
  }

  public boolean sensorErrorHandler(){
    boolean hasFaults = motor.getFault(FaultID.kCANTX) || motor.getFault(FaultID.kCANRX);
    boolean hasStickyFaults = motor.getStickyFault(FaultID.kCANTX) || motor.getStickyFault(FaultID.kCANRX);
    var pose = encoder.getPosition();

    if(pose==0 || pose>2000 || pose<-2000){
      zeroCount++;
    }
    
    var zeroCountFault = zeroCount > 1;
    Logger.getInstance().recordOutput("Wrist/Faults/Zero Count Fault",zeroCountFault);
    Logger.getInstance().recordOutput("Wrist/Faults/Fault", hasFaults);
    Logger.getInstance().recordOutput("Wrist/Faults/Sticky Fault", hasStickyFaults);

    return zeroCountFault || hasFaults || hasStickyFaults;
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

    try {Thread.sleep((long)40.0);} catch(Exception e) {e.printStackTrace();};
  }

  /* Don't move towards the base of the robot if inside it (not good) */
  public double applySoftLimit(double volts) {
    if (WristState.INSIDE_ROBOT.inRange(getArmPose())) {
      if ((Math.signum(volts) == Math.signum(getPose() - 90) && getPose() > -90) || (getPose() < -90 && volts > 0)) {
        Logger.getInstance().recordOutput("Wrist/AntiDestructionTriggered", true);
        return 0;
      }
    }
    Logger.getInstance().recordOutput("Wrist/AntiDestructionTriggered", false);
    return volts;
  }
}
