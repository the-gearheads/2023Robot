package frc.robot.auton;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.wrist.Wrist;

public class AutonChooser {
  private HashMap<String, Command> autons;
  private Swerve swerve;//necessary to create PPSwerveCommand instances
  private SendableChooser<Command> chooser;
  private final String defaultChoice = "N4 Place Then Dock";
  private Wrist wrist;
  private Arm arm;
  private Grabber grabber;

  public AutonChooser(Swerve swerve, Arm arm, Wrist wrist, Grabber grabber) {
    this.swerve = swerve;
    this.arm = arm;
    this.wrist = wrist;
    this.grabber = grabber;
    this.chooser = new SendableChooser<>();
    this.autons = new HashMap<>();

    initializeAutons();
    populateChooser();
    setDefaultAuton();
    SmartDashboard.putData(chooser);
  }

  public void populateChooser() {
    for (Map.Entry<String, Command> auton : autons.entrySet()) {
      chooser.addOption(auton.getKey(), auton.getValue());
    }
  }

  public void setDefaultAuton() {
    if (autons.containsKey(defaultChoice)) {//set default
      chooser.setDefaultOption(defaultChoice, autons.get(defaultChoice));
    } else {//Otherwise, set default to first choice
      DriverStation.reportError("Default Auton Not Defined", false);
      if (autons.size() > 0) {
        Map.Entry<String, Command> firstAuton = autons.entrySet().iterator().next();
        chooser.setDefaultOption(firstAuton.getKey(), firstAuton.getValue());
      } else {
        DriverStation.reportError("No Autons Defined", false);

      }
    }
  }

  public Command getSelectedAuton() {
    Command selectedAuton = chooser.getSelected();
    if (selectedAuton == null) {
      DriverStation.reportError("Selected Auton is null", false);
      if (autons.size() > 0) {
        Map.Entry<String, Command> firstAuton = autons.entrySet().iterator().next();
        selectedAuton = firstAuton.getValue();
      } else {
        DriverStation.reportError("No Autons Defined", false);
        selectedAuton = new PrintCommand("No Auton Selected");
      }
    }
    return selectedAuton;
  }

  public void initializeAutons() {//Here we define auton choices
    Subsystems s = new Subsystems(swerve, wrist, arm, grabber);

    Command InertN4PlaceThenDock = AutonPaths.InertN4PlaceThenDock(s);
    Command InertN1PlaceThenExplore = AutonPaths.InertN1PlaceThenExplore(s);
    Command InertN9PlaceThenExplore = AutonPaths.InertN9PlaceThenExplore(s);
    Command justPlace = AutonPaths.InertN1Place(s);
    // Command doubleCone = AutonPaths.InertN12ConePrime(s);

    // autons.put("2 cone", doubleCone);
    autons.put("NO BUMP Place Then Move", InertN1PlaceThenExplore);
    autons.put("N4 Place Then Dock", InertN4PlaceThenDock);
    autons.put("BUMP Place Then Move", InertN9PlaceThenExplore);
    autons.put("Just Place", justPlace);
  }
}
