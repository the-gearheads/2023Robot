package frc.robot.auton.paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auton.AutonAnnotation;
import frc.robot.auton.AutonRoutine;
import frc.robot.subsystems.Subsystems;

@AutonAnnotation(name = "Test path", variants = {"bruh", "2.0"})
public class TestPath extends AutonRoutine {
  @Override
  public CommandBase getCommand(Subsystems s, String v) {
    if (v.equals("bruh")) {
      return new InstantCommand(() -> {
        DriverStation.reportWarning("aaaaa", false);
      });
    } else {
      return new InstantCommand(() -> {
        DriverStation.reportWarning("bbbbb", false);
      });
    }
  }
}
