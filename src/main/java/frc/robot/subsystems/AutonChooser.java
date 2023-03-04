package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import javax.naming.AuthenticationNotSupportedException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.AutonPaths;
import frc.robot.Constants.AUTON;
import frc.robot.commands.drive.AutoBalance;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.wrist.Wrist;

public class AutonChooser {
  private HashMap<String, Command> autons;
  private Swerve swerve;//necessary to create PPSwerveCommand instances
  private SendableChooser<Command> chooser;
  private final String defaultChoice = "Left, 2 Cones, Charging Station";
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
    //Left Most Cone Node -> Left Most Game Piece -> Left Most Cone Node -> Charging Station 
    Command InertN1TwoConePath = AutonPaths.InertN1TwoConePath(s);
    Command InertN1GrabThenDock = AutonPaths.InertN1GrabThenDock(s);
    Command InertN1PlaceThenDock = AutonPaths.InertN1PlaceThenDock(s);
    Command InertN9TwoConePath = AutonPaths.InertN9TwoConePath(s);
    Command InertN9GrabThenDock = AutonPaths.InertN9GrabThenDock(s);
    Command InertN9PlaceThenDock = AutonPaths.InertN9PlaceThenDock(s);
    Command InertN4PlaceThenDock = AutonPaths.InertN4PlaceThenDock(s);

    autons.put("N4 Place Then Dock", InertN4PlaceThenDock);
    autons.put("N1 TwoCone", InertN1TwoConePath);
    autons.put("N1 Grab Then Dock", InertN1GrabThenDock);
    autons.put("N1 Place Then Dock", InertN1PlaceThenDock);
    autons.put("N9 TwoCone", InertN9TwoConePath);
    autons.put("N9 Grab Then Dock", InertN9GrabThenDock);
    autons.put("N9 Place Then Dock", InertN9PlaceThenDock);

    // //Forward 1 Meter
    // Command debugForward = AutonPaths.getCommandForPath("Debug_Forward", true, AUTON.SLOW_CONSTRAINTS, swerve);
    // autons.put("Debug Forward", debugForward);

    Command autoBalance = new AutoBalance(swerve);
    autons.put("Auto Balance", autoBalance);

  }
}
