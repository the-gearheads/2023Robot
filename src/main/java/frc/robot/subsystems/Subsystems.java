package frc.robot.subsystems;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.wrist.Wrist;

/* Container class for all subsystems */
public class Subsystems {
  public Swerve swerve;
  public Wrist wrist;
  public Arm arm;
  public Vision vision;
  public Grabber grabber;

  public Subsystems(Swerve swerve, Wrist wrist, Arm arm, Vision vision, Grabber grabber) {
    this.swerve = swerve;
    this.wrist = wrist;
    this.arm = arm;
    this.vision = vision;
    this.grabber = grabber;
  }
}
