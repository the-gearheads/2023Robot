package frc.robot.annotations;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;

/**
 * Publish this variable to NetworkTables, requires instantiating the class and calling putData on it
 * due to a reliance on Sendable. Will auto update bidirectionally. 
 */
@Retention(RetentionPolicy.RUNTIME)
public @interface NTPublish {}