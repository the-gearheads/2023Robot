package frc.robot.annotations;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

/**
 * Set a double based on the type of robot currently running. 
 */
@Retention(RetentionPolicy.RUNTIME)
public @interface MultiRobotDouble {
  double sim();
  double real();
}