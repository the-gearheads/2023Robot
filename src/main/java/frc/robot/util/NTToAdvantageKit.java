package frc.robot.util;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NTToAdvantageKit {
    private static void logNTNumber(String name) {
        Logger.getInstance().recordOutput(name, SmartDashboard.getNumber(name, 0));
    }

    public static void periodic() {
        logNTNumber("PPSwerveControllerCommand_rotationError");
        logNTNumber("PPSwerveControllerCommand_xError");
        logNTNumber("PPSwerveControllerCommand_yError");
    }   
}
