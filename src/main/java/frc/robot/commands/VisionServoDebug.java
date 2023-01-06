
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class VisionServoDebug extends CommandBase {
  /** Creates a new TrackAprilTags. */
  private Vision vision;
  private Timer timer;
  private SwerveSubsystem swerveSubsystem;

  public VisionServoDebug(Vision vision, SwerveSubsystem swerveSubsystem) {
    this.vision = vision;
    this.swerveSubsystem = swerveSubsystem;
    timer = new Timer();
    addRequirements(vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    SmartDashboard.putNumber("ServoDebug/Desired Servo Field Relative Angle", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredFieldRelativeAngle = SmartDashboard.getNumber("ServoDebug/Desired Servo Field Relative Angle", 0);
    double gyroAngle = -swerveSubsystem.getContinuousGyroAngle();
    double desiredRobotRelativeAngle = desiredFieldRelativeAngle - gyroAngle;
    desiredRobotRelativeAngle += Constants.Vision.SERVO_OFFSET;
    double desiredRobotRelativeAngleMod360 = desiredRobotRelativeAngle % 360 < 0
        ? desiredRobotRelativeAngle % 360 + 360
        : desiredRobotRelativeAngle % 360;
    if (desiredRobotRelativeAngleMod360 <= 180) {
      vision.setServoAngle(desiredRobotRelativeAngleMod360);
    }

    // debug telemetry
    SmartDashboard.putNumber("ServoDebug/Gyro Angle", gyroAngle);
    SmartDashboard.putNumber("ServoDebug/Desired Robot-Relative Angle", desiredRobotRelativeAngle);
    SmartDashboard.putNumber("ServoDebug/Desired Robot-Relative Angle % 360", desiredRobotRelativeAngleMod360);
  }

  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
