package frc.robot.auton;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

@Retention(RetentionPolicy.RUNTIME)
public @interface AutonAnnotation {
  public String name();

  public String[] variants() default {};
}
