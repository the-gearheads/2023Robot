package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.Controllers;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;

public class WaitForDriveAwayCommand extends CommandBase {
  /** Creates a new AutoBalance. */
  Swerve swerve;
  Leds leds;

  public WaitForDriveAwayCommand(Swerve swerve, Leds leds) {
    this.swerve = swerve;
    this.leds = leds;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setState(LedState.FLASH_GREEN);
  }

  // Called once the commak nd ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.resetState();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Controllers.driverController.getXMoveAxis() < 0;
  }
}
