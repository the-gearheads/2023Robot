// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ARM;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristState;

public class Throw extends CommandBase {
  private Arm arm;
  private Wrist wrist;
  private Grabber grabber;
  private ThrowState releaseState;
  private ThrowState initState;
  private ThrowState finalState;
  private int phase;
  private ThrowState lastState;

  /** Creates a new Throw. */
  public Throw(Arm arm, Wrist wrist, Grabber grabber, double releaseArmPose, double releaseArmSpeed, double releaseWristPose) {
    this.arm = arm;
    this.wrist = wrist;
    this.grabber=grabber;
    this.releaseState = new ThrowState(releaseArmPose, releaseArmSpeed, releaseWristPose);
    addRequirements(arm, wrist);
  }

  @Override
  public void initialize() {
    //calc init state
    // v^2=2ax
    // x = v^2/(2a)
    var accSign = Math.signum(releaseState.armPose - arm.getPose());
    var acc = accSign * ARM.ARM_CONSTRAINTS.maxAcceleration;
    var deltaPose = releaseState.armSpeed/(2*acc);

    var initArmPose = arm.getPose() + deltaPose;
    this.initState = new ThrowState(initArmPose,0,this.releaseState.wristPose);

    this.finalState = new ThrowState(this.releaseState.armPose, 0, this.releaseState.wristPose);

    this.phase=1;

    this.lastState=new ThrowState(arm.getPose(), 0, wrist.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
      if(reached(this.initState)){ 
        phase++;
        this.lastState=getCurrentState();
      }
      break;
      case 3:
      setThrowState(this.finalState);
      if(reached(this.initState)){ 
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
    var moveDir = Math.signum(state.armPose - lastState.armPose);
    if(moveDir>0){
      return arm.getPose() >= state.armPose;
    }else{
      return arm.getPose() <= state.armPose;
    }
  }

  private ThrowState getCurrentState(){
    return new ThrowState(arm.getPose(), arm.getVelocity(), wrist.getPose());
  }

  @Override
  public void end(boolean interrupted){
    this.arm.setPoseGoal(this.arm.getPose());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return phase==4;
  }

  public class ThrowState{
    private double wristPose;
    private double armSpeed;
    private double armPose;

    public ThrowState(double armPose, double armSpeed, double wristPose){
      this.armPose=armPose;
      this.armSpeed=armSpeed;
      this.wristPose=wristPose;
    }
  }
}
