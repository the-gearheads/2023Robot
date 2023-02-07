package frc.robot.subsystems;

import java.sql.Driver;
import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.AUTON;
import frc.robot.subsystems.drive.Swerve;

public class AutonChooser {
    private HashMap<String, Command> autons;
    private Swerve swerve;//necessary to create PPSwerveCommand instances
    private SendableChooser<Command> chooser;
    private final String defaultChoice="";

    public AutonChooser(Swerve swerve){
        this.swerve=swerve;
        this.chooser = new SendableChooser<>();

        initializeAutons();
        populateChooser();
        setDefaultAuton();
        SmartDashboard.putData(chooser);
    }

    public void populateChooser(){
        for (Map.Entry<String,Command> auton : autons.entrySet()) {
            chooser.addOption(auton.getKey(), auton.getValue());
        }
    }

    public void setDefaultAuton(){
        if(autons.containsKey(defaultChoice)){//set default
            chooser.setDefaultOption(defaultChoice, autons.get(defaultChoice));
        }else{//Otherwise, set default to first choice
            DriverStation.reportError("Default Auton Not Defined", false);
            if(autons.size()>0){
                Map.Entry<String,Command> firstAuton = autons.entrySet().iterator().next();
                chooser.setDefaultOption(firstAuton.getKey(), firstAuton.getValue());
            }else{
                DriverStation.reportError("No Autons Defined", false);
                
            }
        }
    }

    public Command getSelectedAuton(){
        Command selectedAuton = chooser.getSelected();
        if(selectedAuton==null){
            DriverStation.reportError("Selected Auton is null", false);
            if(autons.size()>0){
                Map.Entry<String,Command> firstAuton = autons.entrySet().iterator().next();
                selectedAuton = firstAuton.getValue();
            }else{
                DriverStation.reportError("No Autons Defined", false);
                selectedAuton = new PrintCommand("No Auton Selected");
            }
        }
        return selectedAuton;
    }

    public void initializeAutons(){//Here we define auton choices
        autons = new HashMap<String, Command>();

        //Left Most Cone Node -> Left Most Game Piece -> Left Most Cone Node -> Charging Station 
        Command leftSide2ConesChargingStation = getCommandForPath("Start_To_Game_Piece_1", true, AUTON.FAST_CONSTRAINTS)
        .andThen(getCommandForPath("Game_Piece_1_To_Start", false, AUTON.MID_CONSTRAINTS))
        .andThen(getCommandForPath("Start_To_Charging_Station", false, AUTON.SLOW_CONSTRAINTS))
        .andThen(new InstantCommand(()->{swerve.setX();}));
        autons.put("Left, 2 Cones, Charging Station", leftSide2ConesChargingStation);

        //Forward 1 Meter
        Command debugForward = getCommandForPath("Debug_Forward", true, AUTON.SLOW_CONSTRAINTS);
        autons.put("Debug Forward", debugForward); 
    }


    private Command getCommandForPath(String pathName, boolean resetOdometry, PathConstraints constraints) {
        PathPlannerTrajectory path = PathPlanner.loadPath(pathName, constraints);
        if(path == null) {
            DriverStation.reportError("Failed to load path: " + pathName, true);
            return new InstantCommand(()->{
            DriverStation.reportError("Tried to execute path that failed to load! Path name: " + pathName, true);
            });
        }
        Command forwardCommand = swerve.followTrajectoryCommand(path, resetOdometry, true);
        return forwardCommand;
    }
}
