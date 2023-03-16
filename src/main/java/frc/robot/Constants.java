// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.photonvision.PhotonCamera;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.annotations.*;
import frc.robot.util.vision.SimCamParams;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This class should not be used for any other purpose. All
 * constants should be declared globally (i.e. public static final). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.
 */
public class Constants extends AnnotatedClass {
  public static final boolean SIM_REPLAY_MODE = false;

  public static enum RobotMode {
    SIM, SIM_REPLAY, REAL
  }

  public static RobotMode getMode() {
    if (RobotBase.isReal()) {
      return RobotMode.REAL;
    }
    if (SIM_REPLAY_MODE) {
      return RobotMode.SIM_REPLAY;
    } else {
      return RobotMode.SIM;
    }
  }


  public static final class MECH_PLOT {
    public static final double PLOT_HEIGHT = 90;
    public static final double PLOT_WIDTH = 130;

    public static final double ARM_TOWER_LENGTH = 49;
    public static final double ARM_LENGTH = 39;
    public static final double WRIST_LENGTH = 11.5;
    public static final double CHASSIS_LENGTH = 30;

    public static final double ARM_PIVOT_X = PLOT_WIDTH / 2;
    public static final double ARM_PIVOT_Y = ARM_TOWER_LENGTH;

    public static final double CHASSIS_X = ARM_PIVOT_X - 24;//the chassis extends 24 inches behind the arm tower
    public static final double CHASSIS_Y = 0;

    public static final double ARM_REDUCTION = 200;
    public static final double WRIST_REDUCTION = 80; // DONT CHANGE THIS VALUE!!!!

    public static final double ARM_MASS = Units.lbsToKilograms(6);
    public static final double WRIST_MASS = Units.lbsToKilograms(0.5);//should be 3.5 but whatev

    public static final double[] SIM_WRIST_PID = {3, 0.5, 0.2};
  }

  public static class ARM {
    public static final int ARM_ID = 9;
    public static final double[] ARM_POS_PID = {0.086, 0, 0};
    public static final double[] ARM_VEL_PID = {0.009, 0, 0};
    // ks, kg, kv, ka
    public static final double[] ARM_FF = {0, 0.5, 3, 0};

    @NTPublish
    public static double VELOCITY = 50;
    public static final double POSE_TOLERANCE = 5;
    public static final double ANGLE_OFFSET = -270;
    public static final Constraints ARM_CONSTRAINTS = new Constraints(120, 150);
    public static final Constraints ARM_VEL_CONSTRAINTS = new Constraints(VELOCITY, 10);
    public static final double MAX_ANGLE = 10;
    public static final double MIN_ANGLE = -190;
  }

  public static final class WRIST {
    public static final int WRIST_ID = 10;
    public static final double[] WRIST_PID = {0.135, 0, 0};//0.17


    public static final double[] WRIST_FF = {0, 0.5, 0};
    public static final double ANGLE_OFFSET = -180;
  }

  public static final class GRABBER {
    /* Compressor min and max pressures */
    public static final double MIN_PRESSURE = 100;
    public static final double MAX_PRESSURE = 120;
  }

  public static final class DRIVE {
    // Drive, then steer ids
    public static final int[] FL_IDS = {1, 2};
    public static final int[] FR_IDS = {3, 4};
    public static final int[] RL_IDS = {5, 6};
    public static final int[] RR_IDS = {7, 8};

    // Chassis relative offset (degrees), then duty cycle encoder offset (whatever units they are)
    public static final double[] FL_OFFSETS = {270, 0.199};//-90
    public static final double[] FR_OFFSETS = {0, 0.543};
    public static final double[] RL_OFFSETS = {180, 0.534};
    public static final double[] RR_OFFSETS = {90, 0.033};//-90

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public static final double DRIVE_PINION_TOOTH_COUNT = 14;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVE_GEAR_RATIO = (45.0 * 22) / (DRIVE_PINION_TOOTH_COUNT * 15);

    // Throughbore encoder is directly on the output steer shaft
    public static final double STEER_GEAR_RATIO = 1;

    public static final double DRIVE_POS_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO; // rotations -> gear ratio adjusted rotations -> meters
    public static final double DRIVE_VEL_FACTOR = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO / 60.0; // rpm -> gear ratio adjusted rpm -> meters/min -> meters/sec 

    public static final double DRIVE_FREE_SPEED = 5676 * DRIVE_VEL_FACTOR; // Convert max neo free speed to max free wheel speed

    public static final double STEER_POS_FACTOR = 2 * Math.PI; // rotations -> radians
    public static final double STEER_VEL_FACTOR = 2 * Math.PI * 60; // rpm -> rad/sec

    public static final double DIRECTION_SLEW_RATE = 3; // radians per second
    public static final double MAG_SLEW_RATE = 7.0; // percent per second (1 = 100%)
    public static final double ROT_SLEW_RATE = 7.0; // percent per second (1 = 100%)

    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int STEER_CURRENT_LIMIT = 20;

    // order: p, i, d, f
    public static final double[] DRIVE_PIDF = {0.04, 0, 0, 1 / DRIVE_FREE_SPEED};
    public static final double[] STEER_PIDF = {1, 0, 0, 0};

    public static final Translation2d FL_POS =
        new Translation2d(Units.inchesToMeters(12.25), Units.inchesToMeters(13.25));
    public static final Translation2d FR_POS =
        new Translation2d(Units.inchesToMeters(12.25), Units.inchesToMeters(-13.25));
    public static final Translation2d RL_POS =
        new Translation2d(Units.inchesToMeters(-12.25), Units.inchesToMeters(13.25));
    public static final Translation2d RR_POS =
        new Translation2d(Units.inchesToMeters(-12.25), Units.inchesToMeters(-13.25));

    @NTPublish
    public static double HIGH_LIN_VEL = 5;

    public static double MID_LIN_VEL = 1.5;//set to 2

    public static double LOW_LIN_VEL = 0.5;
    @NTPublish
    public static double HIGH_ROT_VEL = 2.5;

    public static double MID_ROT_VEL = 1.5;

    public static double LOW_ROT_VEL = 0.5;

    @NTPublish
    public static double MAX_MODULE_SPEED = 5;
    @NTPublish
    public static double MAX_TRANSLATIONAL_SPEED = 2;
    @NTPublish
    public static double MAX_ROTATIONAL_SPEED = 1;
  }

  public static class AUTO_ALIGN {
    public static Pose2d FEEDER_MID_POSE = new Pose2d(14.5, 7.5, Rotation2d.fromDegrees(180));
    public static Pose2d FEEDER_END_POSE = new Pose2d(15.5, 7.5, Rotation2d.fromDegrees(180));
    public static PathConstraints FEEDER_CONSTRAINTS = new PathConstraints(2, 1);
  }

  public static class AUTON {
    public static PIDController X_PID = new PIDController(5, 0, 0);//0.5
    public static PIDController Y_PID = new PIDController(5, 0, 0);//0.5
    public static PIDController ROT_PID = new PIDController(1.5, 0, 0);
    public static PathConstraints REALLY_SLOW_CONSTRAINTS = new PathConstraints(0.75, 0.7);
    public static PathConstraints SLOW_CONSTRAINTS = new PathConstraints(2, 1);
    public static PathConstraints MID_CONSTRAINTS = new PathConstraints(3, 1.5);
    public static PathConstraints FAST_CONSTRAINTS = new PathConstraints(7, 3);

    public static PathConstraints DOUBLE_CONE = new PathConstraints(4.5, 2.5);
    public static final PathConstraints DOCK_CONSTRAINTS = SLOW_CONSTRAINTS;

    public static final String EVENT_NAME = "Wayne";

    public static PIDController AUTO_BALANCE_PID = new PIDController(0.025, 0, 00);
  }
  public static final class FIELD_CONSTANTS {
    public static final double WIDTH = 8.02;
    public static final double LENGTH = 15.8496;
    public static final Pose2d DEBUG_GO_TO_DEST = new Pose2d(1.9, 4.9, Rotation2d.fromDegrees(-180));
  }
  public static final class CONTROLLERS {
    public static final double JOYSTICK_DEADBAND = 0.075;
  }

  public static final class LEDS {
    public static final int PORT = 1;
    public static final int LENGTH = 12;
  }

  public static final class VISION {
    public static final HashMap<PhotonCamera, Transform3d> CAMS_AND_TRANS = new HashMap<PhotonCamera, Transform3d>() {
      {
        put(new PhotonCamera("ov9281"), new Transform3d());
        put(new PhotonCamera("lifecam"), new Transform3d(new Translation3d(0, 0.05, 0), new Rotation3d()));
      }
    };

    public static final List<AprilTag> TEST_TAGS = new ArrayList<AprilTag>() {
      {
        var id1Pose = new Pose3d(2.5, 2.5, 0, new Rotation3d());
        var id1Toid8 = new Transform3d(new Translation3d(0, 0.3, 0), new Rotation3d());
        var id8Pose = id1Pose.plus(id1Toid8);

        add(new AprilTag(1, id1Pose));
        add(new AprilTag(8, id8Pose));
      }
    };
    public static final AprilTagFieldLayout TEST_ATFL = new AprilTagFieldLayout(TEST_TAGS, 5, 5);
  }

  public static final class VISION_SIM {
    public static final List<SimCamParams> SIM_CAMS_PARAMS = new ArrayList<SimCamParams>() {
      {
        add(new SimCamParams(360, 9000, 1500, 1500, 0, new Transform3d(new Translation3d(0, 0, 1), new Rotation3d()),
            "target 1"));

        add(new SimCamParams(360, 9000, 1500, 1500, 0, new Transform3d(new Translation3d(0, 1, 1), new Rotation3d()),
            "target 2"));
        add(new SimCamParams(360, 9000, 1500, 1500, 0,
            new Transform3d(new Translation3d(0, 1, 1), new Rotation3d(0, 0, Math.PI)), "target 3"));
      }
    };
  }
}
