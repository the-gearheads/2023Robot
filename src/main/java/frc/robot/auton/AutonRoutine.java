package frc.robot.auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Subsystems;

public class AutonRoutine {
  public Command getCommand(Subsystems s, String variant) {
    return new InstantCommand(() -> {
      DriverStation.reportWarning("[AutonRoutine] Override me!", false);
    });
  }
}
