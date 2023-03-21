// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ARM;
import frc.robot.commands.arm.ThrowState.ThrowPhase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlMode;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;

public class Throw extends CommandBase {
  private static final double wristTolerance = 10;
  private static final double armTolerance = 2;
  private static final double grabber_wait_time = 1;

  private Arm arm;
  private Wrist wrist;
  private Grabber grabber;
  private Leds leds;
  
  private ThrowState initState;
  private ThrowState preReleaseState;
  private ThrowState releaseState;
  private ThrowState finalState;

  private ThrowPhase phase;
  private ThrowState lastState;

  /** Creates a new Throw. */
  public Throw(Arm arm, Wrist wrist, Grabber grabber, Leds leds, ThrowState releaseState) {
    this.arm = arm;
    this.wrist = wrist;
    this.grabber = grabber;
    this.leds = leds;
    this.releaseState = releaseState;
    addRequirements(arm, wrist, grabber, leds);
  }

  @Override
  public void initialize() {
    // calculate pre-release state
    // calculate distance for grabber to open
    // x = v*t
    var preReleaseDeltaPose = releaseState.armSpeed * grabber_wait_time;
    var preReleaseArmPose = releaseState.armPose - preReleaseDeltaPose;
    this.preReleaseState = new ThrowState(preReleaseArmPose, releaseState.armSpeed, releaseState.wristPose);

    // calculate init state
    // calculate min distance for arm to reach pre-release state
    // v^2=2ax
    // x = v^2/(2a)
    var accSign = Math.signum(releaseState.armSpeed);
    var acc = accSign * ARM.ARM_CONSTRAINTS.maxAcceleration;
    var deltaPose = Math.pow(releaseState.armSpeed, 2) / (2 * acc);

    var toleranceDelta = 15 * Math.signum(releaseState.armSpeed);
    var initArmPose = preReleaseState.armPose - deltaPose - toleranceDelta;
    this.initState = new ThrowState(initArmPose, 0, this.releaseState.wristPose);

    // calculate final state (give arm time to slow down)
    var finalDelta = 10 * Math.signum(releaseState.armSpeed);
    var finalArmPose = this.releaseState.armPose + finalDelta;
    this.finalState = new ThrowState(finalArmPose, 0, this.releaseState.wristPose);

    // set fields
    this.phase = ThrowPhase.INIT;
    this.lastState = getCurrentState();

    // led signals (helps for testing)
    leds.setState(LedState.WHITE);

    // logging
    SmartDashboard.putBoolean("throw/on?", true);
    logState(initState, "init");
    logState(releaseState, "release");
    logState(finalState, "final");

    if (!this.arm.inAllowableRange(initState.armPose)) {
      DriverStation.reportWarning("invalid init pose when throwing", true);
      this.cancel();
    } else if (!this.arm.inAllowableRange(finalState.armPose)) {
      DriverStation.reportWarning("invalid final pose when throwing", true);
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // log phase
    SmartDashboard.putString("throw/phase", phase.name());

    // main switch
    switch (phase) {
      case INIT:
        setThrowState(this.initState);
        if (armReached(this.initState, 15) && wristReached(this.initState)) {
          phase = ThrowPhase.PRE_RELEASE;
          this.lastState = getCurrentState();
        }
        break;
      case PRE_RELEASE:
        setThrowState(this.preReleaseState);
        leds.setState(LedState.PURPLE);
        if (armReached(this.preReleaseState)) {
          grabber.open();
          leds.setState(LedState.GREEN);

          phase = ThrowPhase.RELEASE;
          this.lastState = getCurrentState();
          logState(getCurrentState(), "pre-release empirical");
        }
        break;
      case RELEASE:
        arm.setControlMode(ArmControlMode.VEL);
        arm.setVelGoal(this.releaseState.armSpeed);

        leds.setState(LedState.YELLOW);
        if (armReached(this.releaseState)) {
          phase = ThrowPhase.FINAL;
          this.lastState = getCurrentState();
          logState(getCurrentState(), "release empirical");
        }
        break;
      case FINAL:
        setThrowState(this.finalState);
        leds.setState(LedState.WHITE);
        grabber.close();
        if (armReached(this.finalState)) {
          phase = ThrowPhase.END;
          this.lastState = getCurrentState();
        }
        break;
    }
  }

  private void setThrowState(ThrowState state) {
    arm.setControlMode(ArmControlMode.POS);
    var armGoal = new TrapezoidProfile.State(state.armPose, state.armSpeed);
    this.arm.setPoseGoal(armGoal);

    var wristGoal = WristState.getStateWithGoal(state.wristPose);
    this.wrist.setGoal(wristGoal);
  }

  private boolean armReached(ThrowState state, double armTolerance) {
    boolean armPast;
    var armClose = Math.abs(arm.getPose() - state.armPose) < armTolerance;

    var moveDir = Math.signum(state.armPose - lastState.armPose);
    if (moveDir > 0) {
      armPast = arm.getPose() > state.armPose;
    } else {
      armPast = arm.getPose() < state.armPose;
    }

    var armReached = armPast || armClose;
    return armReached;
  }

  private boolean wristReached(ThrowState state, double wristTolerance) {
    return Math.abs(state.wristPose - wrist.getPose()) < wristTolerance;
  }

  private boolean wristReached(ThrowState state) {
    return wristReached(state, wristTolerance);
  }

  private boolean armReached(ThrowState state) {
    return this.armReached(state, Throw.armTolerance);
  }

  private ThrowState getCurrentState() {
    return new ThrowState(arm.getPose(), arm.getVel(), wrist.getPose());
  }

  public void logState(ThrowState state, String path) {
    SmartDashboard.putNumber("throw/" + path + "/arm pose", state.armPose);
    SmartDashboard.putNumber("throw/" + path + "/arm speed", state.armSpeed);
    SmartDashboard.putNumber("throw/" + path + "/wrist pose", state.wristPose);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("throw/on?", false);
    this.arm.setPoseGoal(this.arm.getPose());
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase == ThrowPhase.END;
  }
}
