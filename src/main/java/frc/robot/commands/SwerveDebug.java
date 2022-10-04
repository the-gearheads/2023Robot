package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.SwerveSubsystem;

public class SwerveDebug extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveDebug(SwerveSubsystem subsystem) {
    swerveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.zeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpd = Controllers.activeController.getXMoveAxis();
    var ySpd = Controllers.activeController.getYMoveAxis();
    var rotSpd = Controllers.activeController.getRotateAxis();
    swerveSubsystem.driveFieldRelative(new ChassisSpeeds(xSpd, ySpd, rotSpd));

    SmartDashboard.putBoolean("/Swerve/PerformOptimizations", !Controllers.activeController.getSwerveDebugOptimizeEnabledButton().get());
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
