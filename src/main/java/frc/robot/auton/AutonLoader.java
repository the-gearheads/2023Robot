package frc.robot.auton;

import org.reflections.Reflections;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.CustomSendableChooser;
import static org.reflections.scanners.Scanners.*;
import java.util.Map;


public class AutonLoader {
  public static CustomSendableChooser<Command> chooser = new CustomSendableChooser<>();

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
    Command selectedAuton = chooser.getSelected();
    var autons = chooser.getAutons();
    
    if (selectedAuton == null) {
      DriverStation.reportError("Selected Auton is null", false);
      if (autons.size() > 0) {
        var firstAuton = (Map.Entry<String, Command>) autons.entrySet().iterator().next();
        selectedAuton = firstAuton.getValue();
      } else {
        DriverStation.reportError("No Autons Defined", false);
        selectedAuton = new PrintCommand("No Auton Selected");
      }
    }
    return selectedAuton;
  }
}
