package frc.robot.subsystems;

import java.util.HashMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Swerve;

public class AutonChooser {
    private HashMap<String, Command> choices;
    private Swerve swerve;
    public AutonChooser(Swerve swerve){
        this.swerve=swerve;
    }

    public void initializeChoices(){

    }

}
