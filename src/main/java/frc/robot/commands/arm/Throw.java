// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ARM;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;

public class Throw extends CommandBase {
  private static final double waitTime = 0.2;
  private static final double wristTolerance = 0.5;
  private static final double armTolerance = 0.5;
  private Arm arm;
  private Wrist wrist;
  private Grabber grabber;
  private ThrowState releaseState;
  private ThrowState initState;
  private ThrowState finalState;
  private int phase;
  private ThrowState lastState;
  private boolean waiting;
  private double lastTime;
  private Leds leds;

  /** Creates a new Throw. */
  public Throw(Arm arm, Wrist wrist, Grabber grabber, Leds leds, ThrowState releaseState) {
    this.arm = arm;
    this.wrist = wrist;
    this.grabber=grabber;
    this.leds=leds;
    this.releaseState = releaseState;
    addRequirements(arm, wrist, grabber, leds);
  }

  @Override
  public void initialize() {
    //calc init state

    //calculate min distance for arm to reach release state
    // v^2=2ax
    // x = v^2/(2a)
    var currentPose=arm.getPose(); // remove
    var accSign = Math.signum(releaseState.armSpeed);
    var acc = accSign * ARM.ARM_CONSTRAINTS.maxAcceleration / 20;
    var deltaPose = Math.pow(releaseState.armSpeed,2)/(2*acc);

    var initArmPose = releaseState.armPose - deltaPose;
    this.initState = new ThrowState(initArmPose,0,this.releaseState.wristPose);

    this.finalState = new ThrowState(this.releaseState.armPose, 0, this.releaseState.wristPose);

    this.phase=1;

    this.lastState=new ThrowState(arm.getPose(), 0, wrist.getPose());
    leds.setState(LedState.WHITE);

    SmartDashboard.putBoolean("throw/on?", true);

    logState(initState, "init");
    logState(releaseState, "release");
    logState(finalState, "final");


    this.lastTime=-1;
    this.waiting=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("throw/phase", phase);
    switch(phase){
      case 1:
      setThrowState(this.initState);
      if(reached(this.initState)){ 
        phase++;
        this.lastState=getCurrentState();
      }
      break;
      case 2:
      setThrowState(this.releaseState);
      leds.setState(LedState.GREEN);
      if(reached(this.releaseState)){ 
        phase++;
        this.lastState=getCurrentState();
        grabber.open();
        logState(getCurrentState(), "empirical");
      }
      break;
      case 3:
      var currentTime = Timer.getFPGATimestamp();
      leds.setState(LedState.ORANGE);
      if(!waiting){
        lastTime = currentTime;
        waiting=true;
      }else if(currentTime - lastTime > waitTime){
        this.lastState=getCurrentState();
        phase++;
      }
      break;
      case 4:
      setThrowState(this.finalState);
      leds.setState(LedState.WHITE);
      grabber.close();
      if(reached(this.finalState)){ 
        phase++;
        this.lastState=getCurrentState();
      }
      break;
    }
  }

  private void setThrowState(ThrowState state){
    var armGoal = new TrapezoidProfile.State(state.armPose, state.armSpeed);
    this.arm.setPoseGoal(armGoal);
    
    var wristGoal = WristState.getStateWithGoal(state.wristPose);
    this.wrist.setGoal(wristGoal);
  }

  private boolean reached(ThrowState state) {
    boolean armPast;
    var armClose = Math.abs(arm.getPose() - state.armPose) < armTolerance;
    
    var currentArmPose = arm.getPose();//remove
    var moveDir = Math.signum(state.armPose - lastState.armPose);
    if(moveDir>0){
      armPast = arm.getPose() > state.armPose;
    }else{
      armPast = arm.getPose() < state.armPose;
    }

    var armReached = armPast || armClose;
    var wristReached = Math.abs(state.wristPose - wrist.getPose()) < wristTolerance;

    return wristReached && armReached;
  }

  private ThrowState getCurrentState(){
    return new ThrowState(arm.getPose(), arm.getVel(), wrist.getPose());
  }

  @Override
  public void end(boolean interrupted){
    SmartDashboard.putBoolean("throw/on?", false);
    this.arm.setPoseGoal(this.arm.getPose());
  }

  public void logState(ThrowState state, String path){
    SmartDashboard.putNumber("throw/" + path + "/arm pose", state.armPose);
    SmartDashboard.putNumber("throw/" + path + "/arm speed", state.armSpeed);
    SmartDashboard.putNumber("throw/" + path + "/wrist pose", state.wristPose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase==5;
  }
}
