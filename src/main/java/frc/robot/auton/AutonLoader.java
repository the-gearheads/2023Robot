package frc.robot.auton;

import org.reflections.Reflections;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Subsystems;
import static org.reflections.scanners.Scanners.*;


public class AutonLoader {
  public static SendableChooser<Command> chooser = new SendableChooser<>();

  public static void load(Subsystems s) {
    try {
      /* Get all annotated classes and iterate over them */
      Reflections ref = new Reflections("frc.robot");
      var annotatedClasses = ref.get(SubTypes.of(TypesAnnotated.with(AutonAnnotation.class)).asClass());

      for (Class<?> a : annotatedClasses) {
        var annotation = a.getAnnotation(AutonAnnotation.class);
        /* Actually instantiate it */
        var instance = (AutonRoutine) a.getConstructor().newInstance();
        /* Don't always have variants */
        if (annotation.variants().length == 0) {
          chooser.addOption(annotation.name(), instance.getCommand(s, ""));
          continue;
        }
        /* But if we do, call getCommand for all of them */
        for (String variant : annotation.variants()) {
          chooser.addOption(variant + " " + annotation.name(), instance.getCommand(s, variant));
        }
      }
    } catch (Exception e) {
      /* yes, bad practice */ e.printStackTrace();
    }

    SmartDashboard.putData("New chooser", chooser);
  }

  public static Command getCommand() {
    return chooser.getSelected();
  }
}
