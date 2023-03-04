// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WRIST;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.WristState.WristStateType;
import frc.robot.util.SendableArmFeedforward;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private double goal;
  private CANSparkMax motor = new CANSparkMax(WRIST.WRIST_ID, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
  private RelativeEncoder relativeEncoder;
  protected PIDController pid = new PIDController(WRIST.WRIST_PID[0], WRIST.WRIST_PID[1], WRIST.WRIST_PID[2]);
  private SendableArmFeedforward ff =
      new SendableArmFeedforward(WRIST.WRIST_FF[0], WRIST.WRIST_FF[1], WRIST.WRIST_FF[2]);
  private Arm arm;
  private WristStateType controlState = WristStateType.DEFAULT;

  public Wrist(Arm arm) {
    this.arm = arm;
    configure();

    SmartDashboard.putData("Wrist/ff", ff);
    SmartDashboard.putData("Wrist/pid", pid);
  }

  public void setControlState(WristStateType wristStateType) {
    controlState = wristStateType;
  }

  public double getGoal() {
    return goal;
  }

  public void setGoal(double newGoal) {
    goal = newGoal;
  }

  public double getPose() {
    return encoder.getPosition() + Constants.WRIST.ANGLE_OFFSET;
  }

  /**
   The code calculates the number of full 360-degree wraps using the relativeEncoder.getPosition() method, and then calculates the relative encoder position modulo 360 to get the position within the current 360-degree wrap. It adjusts the modulo value by adding 360 degrees and subtracting 1 from the number of wraps if the modulo value is negative.

The code then checks if the relative encoder position is greater than 180 degrees and the absolute encoder position is less than 180 degrees, or if the relative encoder position is less than 180 degrees and the absolute encoder position is greater than 180 degrees. If either condition is true, it adjusts the number of wraps accordingly.

Finally, the code multiplies the number of wraps by 360 degrees and adds the absolute encoder position to get the continuous angle.

However, it's important to note that without knowledge of the specific system requirements, it is impossible to determine if the code meets all requirements or if there are any edge cases that it doesn't handle. It is always recommended to thoroughly test the code and ensure that it works correctly in all possible scenarios before using it in a production environment.
 */
  public double getCtsPose(){//dont touch
    int numWraps = (int) (relativeEncoder.getPosition() / 360);

    double relativeEncoderPoseMod360 = relativeEncoder.getPosition() % 360;
    if(relativeEncoderPoseMod360<0){
      relativeEncoderPoseMod360+=360;
      numWraps-=1;
    } 

    if(relativeEncoderPoseMod360 > 180 && getPose() < 180){
      numWraps+=1;
    }else if(relativeEncoderPoseMod360 < 180 && getPose() > 180){
      numWraps-=1;
    }

    int ctsOffset = numWraps*360;
    return ctsOffset + getPose();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  //In case a wrist command needs to access arm pose (don't want to give it entire arm subsystem)
  public double getArmPose() {
    return arm.getPose();
  }

  public void setVoltage(double volts) {
    // motor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    setGoalByType(controlState);
    setGoalByType(WristStateType.OVERRIDE);
    double currentPose = getPose();

    double pidval = pid.calculate(currentPose, goal);
    double ffval = ff.calculate(currentPose, 0);

    Logger.getInstance().recordOutput("Wrist/Pose", currentPose);
    Logger.getInstance().recordOutput("Wrist/Cts Pose", getCtsPose());
    Logger.getInstance().recordOutput("Wrist/Vel", getVelocity());
    Logger.getInstance().recordOutput("Wrist/Goal", goal);
    Logger.getInstance().recordOutput("Wrist/Error", pid.getPositionError());
    Logger.getInstance().recordOutput("Wrist/PIDVal", pidval);
    Logger.getInstance().recordOutput("Wrist/FFVal", ffval);
    Logger.getInstance().recordOutput("Wrist/Appliedvolts", pidval + ffval);
    Logger.getInstance().recordOutput("Wrist/Current Command",
        this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "");

    setVoltage(applySoftLimit(ffval + pidval));
  }

  public void setGoalByType(WristStateType wristStateType) { // check what range the arm is in and set the wrist accordingly
    double armPos = arm.getPose();
    for (WristState wristState : WristState.values()) {
      if (wristState.type == wristStateType && wristState.inRange(armPos)) {
        setGoal(wristState.getWristGoal(armPos));
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

    // don't touch this either
    relativeEncoder = motor.getEncoder();
    relativeEncoder.setPositionConversionFactor(360.0 / Constants.MECH_PLOT.WRIST_REDUCTION);
    relativeEncoder.setVelocityConversionFactor(360.0 / Constants.MECH_PLOT.WRIST_REDUCTION);
    var startPose = getPose();
    if(startPose < -90) startPose+=360;
    relativeEncoder.setPosition(startPose);

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
