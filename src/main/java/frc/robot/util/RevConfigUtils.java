package frc.robot.util;

import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;

public class RevConfigUtils {
  
  /**
   * Runs a rev motor controller configure function with retry support.
   * @param configureFunc Function that returns an ArrayList of REVLibErrors
   * @param subsystemName Name of subsystem (or motor)
   * @param maxRetries Max number of retries (default 5)
   */
  public static void configure(Supplier<ArrayList<REVLibError>> configureFunc, String subsystemName, int maxRetries) {
    /* Loop for maxRetries */
    for(int retries = 0; retries < maxRetries; retries++) {
      var errors = configureFunc.get();
      boolean reconfigure = false;
      /* Loop over all errors */
      for(int i = 0; i < errors.size(); i++) {
        if (errors.get(i) != REVLibError.kOk) {
          reconfigure = true;
          DriverStation.reportError("Configure function error " + errors.get(i).name() + " at i=" + i + "in subsystem " + subsystemName
                                  + "(retry " + retries + "/" + maxRetries + ")", true);

          /* Not super useful or detailed, i just want this to also be in NT and logs */
          Logger.getInstance().recordOutput("Errors/" + subsystemName + errors.get(i).name(), true);
        }
      }
      /* Don't retry if no errors */
      if (!reconfigure) {break;}
    }
  }

  /**
   * Runs a rev motor controller configure function with retry support.
   * @param configureFunc Function that returns an ArrayList of REVLibErrors
   * @param subsystemName Name of subsystem (or motor)
   */
  public static void configure(Supplier<ArrayList<REVLibError>> configureFunc, String subsystemName) {
    configure(configureFunc, subsystemName, 3);
  }
}
