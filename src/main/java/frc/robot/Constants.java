// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.annotations.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants extends AnnotatedClass {
  public static boolean simReplayMode = false;

  public static enum RobotMode {
    SIM, SIM_REPLAY, REAL
  }

  public static RobotMode getMode() {
    if(Robot.isReal()) {
      return RobotMode.REAL;
    }
    if(simReplayMode) {
      return RobotMode.SIM_REPLAY;
    } else {
      return RobotMode.SIM;
    }
  }
  public static class Drivetrain {
  
    // Drive, then steer ids
    public static int[] FL_IDS = {1, 2};
    public static int[] FR_IDS = {3, 4};
    public static int[] RL_IDS = {5, 6};
    public static int[] RR_IDS = {7, 8};

    // Chassis relative offset (degrees), then duty cycle encoder offset (whatever units they are)
    public static final double[] FL_OFFSETS = {270,   0.199};//-90
    public static final double[] FR_OFFSETS = {0,   0.543};
    public static final double[] RL_OFFSETS = {180, 0.534};
    public static final double[] RR_OFFSETS = {90,   0.033};//-90

    public static double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public static double DRIVE_PINION_TOOTH_COUNT = 14;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static double DRIVE_GEAR_RATIO = (45.0 * 22) / (DRIVE_PINION_TOOTH_COUNT * 15);

    // Throughbore encoder is directly on the output steer shaft
    public static double STEER_GEAR_RATIO = 1;

    public static double DRIVE_POS_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO; // rotations -> gear ratio adjusted rotations -> meters
    public static double DRIVE_VEL_FACTOR =  WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO / 60.0; // rpm -> gear ratio adjusted rpm -> meters/min -> meters/sec 

    public static double DRIVE_FREE_SPEED = 5676 * DRIVE_VEL_FACTOR * 1.2; // Convert max neo free speed to max free wheel speed

    public static double STEER_POS_FACTOR = 2 * Math.PI; // rotations -> radians
    public static double STEER_VEL_FACTOR = 2 * Math.PI * 60; // rpm -> rad/sec

    public static int DRIVE_CURRENT_LIMIT = 40;
    public static int STEER_CURRENT_LIMIT = 20;

    // order: p, i, d, f
    public static double[] DRIVE_PIDF = {0.04, 0, 0, 1 / DRIVE_FREE_SPEED};
    public static double[] STEER_PIDF = {1, 0, 0, 0};

    public static Pose2d zeroPos = new Pose2d(0, 0, new Rotation2d(0));
    public static Transform2d zeroTransform = new Transform2d(new Translation2d(0,0), new Rotation2d(0));

    public static Translation2d FL_POS = new Translation2d(Units.inchesToMeters(12.25), Units.inchesToMeters(13.25));
    public static Translation2d FR_POS = new Translation2d(Units.inchesToMeters(12.25), Units.inchesToMeters(-13.25));
    public static Translation2d RL_POS = new Translation2d(Units.inchesToMeters(-12.25), Units.inchesToMeters(13.25));
    public static Translation2d RR_POS = new Translation2d(Units.inchesToMeters(-12.25), Units.inchesToMeters(-13.25));

    @NTPublish
    public static double MAX_LIN_VEL = 2;//set to 2
    @NTPublish
    public static double MAX_ROT_VEL = 1;

    @NTPublish
    public static double MAX_MODULE_SPEED = 5;
    @NTPublish
    public static double MAX_TRANSLATIONAL_SPEED = 2;
    @NTPublish
    public static double MAX_ROTATIONAL_SPEED = 1;

    public static class Auton {
      public static PIDController X_PID = new PIDController(2.5, 0, 0);
      public static PIDController Y_PID = new PIDController(2.5, 0, 0);
      public static PIDController ROT_PID = new PIDController(1, 0, 0);
      public static PathConstraints CONSTRAINTS = new PathConstraints(1, 1.5);
    }
  }
  public static final class FieldConstants{
    public static double width=50;
    public static double length=25;
    public static Pose2d GRID_8 = new Pose2d(2.8, 1.05, Rotation2d.fromDegrees(180));
  }
  public static class Vision{
    public static Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(-9), Units.inchesToMeters(0.5), Units.inchesToMeters(20)), new Rotation3d());
  }
  public static class Controllers {
    public static double JOYSTICK_DEADBAND = 0.1;
  }
}
