package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AUTON;
import frc.robot.subsystems.drive.Swerve;

public class AutoBalDriveToPivot extends CommandBase {
    private Swerve swerve;
  
    /** Creates a new AutoBalance. */
    public AutoBalDriveToPivot(Swerve swerve) {
      this.swerve = swerve;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerve.drive(new ChassisSpeeds(AUTON.AUTO_BALANCE_VEL, 0, 0)); // 2.5 m/s
    }

    // Called once the commak nd ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double roll = swerve.getRoll();
        if (roll < AUTON.AUTO_BALANCE_PIVOT_ROLL) {
            return true;
        }
        return false;
  }
}
