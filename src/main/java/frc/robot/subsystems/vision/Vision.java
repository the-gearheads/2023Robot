// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TrackAprilTags;

public class Vision extends SubsystemBase {
  private final Servo servo;
  public Vision() {
    servo = new Servo(0);
    setDefaultCommand(new TrackAprilTags(this));
  }
  public double getServoAngle(){
    return servo.getAngle();
  }
  public void setServoAngle(double angle){
    servo.setAngle(angle);
  }
}
