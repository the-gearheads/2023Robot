// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.Vision;

public class TrackAprilTags extends CommandBase {
  /** Creates a new TrackAprilTags. */
  private Vision vision;
  private double timeToRotate180Deg=0.6;
  private Timer timer;
  public TrackAprilTags(Vision vision) {
    this.vision=vision;
    timer=new Timer();
    addRequirements(vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isAprilTagInView = false;//replace with method from vision that checks if there are apriltags in view
    if(isAprilTagInView){
      //follow april tag
      double aprilTagAngleInFrame=0;//replace with mehod from vision that returns the angle of the tag within view of the camera
      vision.setServoAngle((vision.getServoAngle()-aprilTagAngleInFrame)%180);
    }else{
      wander();
    }
  }
  public void wander(){
    if(timer.get()>timeToRotate180Deg){
      vision.setServoAngle(MathUtil.applyDeadband(vision.getServoAngle()-180,1)==0?0:180);
      timer.reset();
      timer.start();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
