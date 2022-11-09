package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    SmartDashboard.putNumber("/Swerve/Debug/PID/F",Constants.Drivetrain.STEER_F);
    SmartDashboard.putNumber("/Swerve/Debug/PID/P",Constants.Drivetrain.STEER_P);
    SmartDashboard.putNumber("/Swerve/Debug/PID/I",Constants.Drivetrain.STEER_I);
    SmartDashboard.putNumber("/Swerve/Debug/PID/D",Constants.Drivetrain.STEER_D);
    SmartDashboard.putNumber("/Swerve/Debug/AngleOffsets/FR",Constants.Drivetrain.FR_OFFSET.getDegrees());
    SmartDashboard.putNumber("/Swerve/Debug/AngleOffsets/FL",Constants.Drivetrain.FL_OFFSET.getDegrees());
    SmartDashboard.putNumber("/Swerve/Debug/AngleOffsets/RR",Constants.Drivetrain.RR_OFFSET.getDegrees());
    SmartDashboard.putNumber("/Swerve/Debug/AngleOffsets/RL",Constants.Drivetrain.RL_OFFSET.getDegrees());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xSpd = Controllers.activeController.getXMoveAxis();
    var ySpd = Controllers.activeController.getYMoveAxis();
    var rotSpd = Controllers.activeController.getRotateAxis();
    swerveSubsystem.driveFieldRelative(new ChassisSpeeds(xSpd, ySpd, rotSpd));

    SmartDashboard.putBoolean("/Swerve/PerformOptimizations", !Controllers.activeController.getSwerveDebugOptimizeEnabledButton().get());
    setPIDConstants();
    setAngleOffsets();
  }
  private void setPIDConstants(){
    double kF=SmartDashboard.getNumber("/Swerve/PID/F",Constants.Drivetrain.STEER_F);
    double kP=SmartDashboard.getNumber("/Swerve/PID/P",Constants.Drivetrain.STEER_P);
    double kI=SmartDashboard.getNumber("/Swerve/PID/I",Constants.Drivetrain.STEER_I);
    double kD=SmartDashboard.getNumber("/Swerve/PID/D",Constants.Drivetrain.STEER_D);
    swerveSubsystem.setPIDConstants(kF, kP, kI, kD);
  }
  private void setAngleOffsets(){
    Rotation2d FR = new Rotation2d(Math.PI/180*SmartDashboard.getNumber("/Swerve/Debug/AngleOffsets/FR",Constants.Drivetrain.FR_OFFSET.getDegrees()));
    Rotation2d FL = new Rotation2d(Math.PI/180*SmartDashboard.getNumber("/Swerve/Debug/AngleOffsets/FL",Constants.Drivetrain.FL_OFFSET.getDegrees()));
    Rotation2d RR = new Rotation2d(Math.PI/180*SmartDashboard.getNumber("/Swerve/Debug/AngleOffsets/RR",Constants.Drivetrain.RR_OFFSET.getDegrees()));
    Rotation2d RL = new Rotation2d(Math.PI/180*SmartDashboard.getNumber("/Swerve/Debug/AngleOffsets/RL",Constants.Drivetrain.RL_OFFSET.getDegrees()));
    swerveSubsystem.setAngleOffsets(new Rotation2d[]{FR,FL,RR,RL});
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
