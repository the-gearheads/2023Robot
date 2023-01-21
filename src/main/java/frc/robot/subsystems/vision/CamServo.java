// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CamServo extends SubsystemBase {
  public final Servo servo;
  private double currentAngle;
  private double lastTime;
  private double goal; //Desired Angle that we want to ultimately reach
  private double setpoint; //Smaller goal to control speed
  
  /** Creates a new CamServo. */
  public CamServo() {
    servo = new Servo(0);
    currentAngle=0;
    goal=currentAngle;
    setpoint=goal;
    lastTime=Timer.getFPGATimestamp();
  }

  public double getGoal(){return goal;}
  public void setGoal(double angle){goal=angle;}
  
  public double getCurrentAngle(){return currentAngle;}

  private void updateCurrentAngle(){
    double deltaTime=Timer.getFPGATimestamp()-lastTime;
    lastTime=Timer.getFPGATimestamp();

    double direction=this.setpoint>this.currentAngle?1:-1;
    double deltaAngle=direction*Constants.Vision.SERVO_SPEED*deltaTime;//direction*speed*time
    this.currentAngle+=deltaAngle;

    if((direction>0 && this.currentAngle>this.setpoint)
     ||(direction<0 && this.currentAngle<this.setpoint)){
      this.currentAngle=this.setpoint;
     }
  }

  private void updateSetpoint(){
    double direction = this.goal>this.currentAngle?1:-1;

    double deltaAngle=direction*Constants.Vision.SERVO_SPEED*0.02;
    this.setpoint=this.currentAngle+deltaAngle;
    
    if((direction>0 && this.setpoint>this.goal)
    || (direction<0 && this.setpoint<this.goal)){
      this.setpoint = this.goal;
    }
  }

  private void printTelemetry(){
    SmartDashboard.putNumber("Cam Servo/Current Angle", this.currentAngle);
    SmartDashboard.putNumber("Cam Servo/Setpoint", this.currentAngle);
    SmartDashboard.putNumber("Cam Servo/Goal", this.currentAngle);
  }

  @Override
  public void periodic() {
    updateCurrentAngle();
    updateSetpoint();
    printTelemetry();
  }
}
