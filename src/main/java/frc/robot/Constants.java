// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import org.photonvision.PhotonCamera;
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

    public static final double[] SIM_WRIST_PID = {8, 0.5, 1};
    public static final double[] SIM_ARM_VEL_PID = {0.009, 0, 0};
    public static final Constraints SIM_ARM_CONSTRAINTS = new Constraints(200, 300);
  }

  public static class ARM {
    public static final int ARM_ID = 9;
    public static final double[] ARM_POS_PID = {0.086, 0.001, 0};
    public static final double[] ARM_VEL_PID = {0.009, 0, 0};
    // ks, kg, kv, ka
    public static final double[] ARM_FF = {0, 0.5, 3, 0};

    @NTPublish
    public static double VELOCITY = 50;
    public static final double POSE_TOLERANCE = 5;
    public static final double ANGLE_OFFSET = -270;
    public static final Constraints ARM_CONSTRAINTS = new Constraints(120, 150);
    public static final Constraints ARM_VEL_CONSTRAINTS = new Constraints(150, 150);// change to 50, 10
    public static final double MAX_ANGLE = 10;
    public static final double MIN_ANGLE = -190;
  }

  public static final class WRIST {
    public static final int WRIST_ID = 10;
    public static final double[] WRIST_PID = {0.135, 0, 0};//0.17


    public static final double[] WRIST_FF = {0, 0.5, 0};
    public static final double ANGLE_OFFSET = 180;//-180
    public static final double WRAP_RANGE_SPEED = 2;
    public static final double WRAP_RANGE_UPPER_BOUND = 135;
    public static final double WRAP_RANGE_LOWER_BOUND = -135;
  }

  public static final class GRABBER {
    /* Compressor min and max pressures */
    public static final double MIN_PRESSURE = 100;
    public static final double MAX_PRESSURE = 120;
    public static final int GRABBER_SWITCH_ID = 2;
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

    public static final double DRIVE_PINION_TOOTH_COUNT = 13;
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
    public static double HIGH_LIN_VEL = 10;

    public static double MID_LIN_VEL = 2.6;//set to 2

    public static double LOW_LIN_VEL = 1.5;//testing: 0.15
    @NTPublish
    public static double HIGH_ROT_VEL = 7.5;

    public static double MID_ROT_VEL = 1;

    public static double LOW_ROT_VEL = 0.5;//testing: 0.3

    @NTPublish
    public static double MAX_MODULE_SPEED = 5;
    @NTPublish
    public static double MAX_TRANSLATIONAL_SPEED = 2;
    @NTPublish
    public static double MAX_ROTATIONAL_SPEED = 1;
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

    public static final String EVENT_NAME = "Macomb";

    public static PIDController AUTO_BALANCE_PID = new PIDController(0.021, 0, 00);
    public static double AUTO_BALANCE_VEL = 0.5;
    public static double AUTO_BALANCE_PIVOT_ROLL = 7.5;
  }
  public static final class FIELD_CONSTANTS {
    public static final double WIDTH = 8.02;
    public static final double LENGTH = 15.8496;
    public static final Pose2d DEBUG_GO_TO_DEST = new Pose2d(1.9, 4.9, Rotation2d.fromDegrees(-180));

    public enum FieldType {
      SHOP, REAL, TEST;
    }

    public static final FieldType FIELD_TYPE = FieldType.SHOP;
  }
  public static final class CONTROLLERS {
    public static final double JOYSTICK_DEADBAND = 0.075;
  }

  public static final class LEDS {
    public static final int PORT = 9;
    /* Rough estimate, refine later */
    public static final int LENGTH = 260;
  }

  public static final class ROBOT {
    public static final double ROBOT_X = Units.inchesToMeters(30.25);//without bumpers
    public static final double ROBOT_Y = Units.inchesToMeters(28.5);//without bumpers
    public static final double ROBOT_X_WITH_BUMPER = Units.inchesToMeters(36);
    public static final double ROBOT_Y_WITH_BUMPER = Units.inchesToMeters(34.5);
  }

  public static final class VISION {

    public static final double CORNER_THRESHOLD = 3;
    public static final double X_RES = 640;
    public static final double Y_RES = 400;
    public static final HashMap<PhotonCamera, Transform3d> CAMS_AND_TRANS = new HashMap<PhotonCamera, Transform3d>() {
      {
        //@formatter:off
        var robotKnownLocForwardCams = new Pose3d(new Pose2d(
          1.369 + Units.inchesToMeters(40) + (Constants.ROBOT.ROBOT_X_WITH_BUMPER/2),
          4.748 - (Constants.ROBOT.ROBOT_Y_WITH_BUMPER / 2),
          Rotation2d.fromDegrees(180)
        ));
        var robotKnownLocBackwardCams = new Pose3d(new Pose2d(
          1.369 + Units.inchesToMeters(40) + (Constants.ROBOT.ROBOT_X_WITH_BUMPER/2),
          4.748 - (Constants.ROBOT.ROBOT_Y_WITH_BUMPER / 2),
          Rotation2d.fromDegrees(0)
        ));
        var forwardLeftPose = new Pose3d(
          2.57,
          4.14,
          1.374,
          new Rotation3d(
            Units.degreesToRadians(-2.05),//roll
            Units.degreesToRadians(39.842),//pitch
            Units.degreesToRadians(150.304)//yaw
          )
        );
        var forwardRightPose = new Pose3d(
          2.509,
          4.444,
          1.342,
          new Rotation3d(
            Units.degreesToRadians(3.889),//roll
            Units.degreesToRadians(39.544),//pitch
            Units.degreesToRadians(-145.613)//yaw
          )
        );
        var backwardLeftPose = new Pose3d(
          3.228,
          4.411,
          1.367,
          new Rotation3d(
            Units.degreesToRadians(-1.844),//roll
            Units.degreesToRadians(35.411),//pitch
            Units.degreesToRadians(-160.7)//yaw
          )
        );
        var backwardRightPose = new Pose3d(
          3.231,
          4.237,
          1.375,
          new Rotation3d(
            Units.degreesToRadians(1.513),//roll
            Units.degreesToRadians(35.867),//pitch
            Units.degreesToRadians(163.141)//yaw
          )
        );

        put(new PhotonCamera("Forward Left"), forwardLeftPose.minus(robotKnownLocForwardCams));
        put(new PhotonCamera("Forward Right"), forwardRightPose.minus(robotKnownLocForwardCams));
        put(new PhotonCamera("Backward Left"), backwardLeftPose.minus(robotKnownLocBackwardCams));
        put(new PhotonCamera("Backward Right"), backwardRightPose.minus(robotKnownLocBackwardCams));

        var robotTip = new Transform3d(
            new Translation3d(
              Units.inchesToMeters(12.9 - 1), 
              Units.inchesToMeters(12.875), 
              Units.inchesToMeters(48.1)),
            new Rotation3d(0, 0, 0));

        var forwardLeftDelta = new Transform3d(
            new Translation3d(
              Units.inchesToMeters(2), 
              Units.inchesToMeters(-6.5), 
              Units.inchesToMeters(0.3)),
            new Rotation3d(
              Units.degreesToRadians(0), 
              Units.degreesToRadians(40),
              Units.degreesToRadians(-30)
              ));

        var backwardLeftDelta = new Transform3d(
            new Translation3d(
              Units.inchesToMeters(0), 
              Units.inchesToMeters(-8.5), 
              Units.inchesToMeters(0.3)),
            new Rotation3d(
              Units.degreesToRadians(0), 
              Units.degreesToRadians(-180-35),
              Units.degreesToRadians(200)
             ));

        var backwardRightDelta = new Transform3d(
            new Translation3d(
              Units.inchesToMeters(0), 
              Units.inchesToMeters(-17), 
              Units.inchesToMeters(0.3)),
            new Rotation3d(
              Units.degreesToRadians(0), 
              Units.degreesToRadians(-180-35.5),
              Units.degreesToRadians(160)
              ));

        var forwardRightDelta = new Transform3d(
            new Translation3d(
              Units.inchesToMeters(2), 
              Units.inchesToMeters(-19), 
              Units.inchesToMeters(0.3)),
            new Rotation3d(
              Units.degreesToRadians(0), 
              Units.degreesToRadians(39),
              Units.degreesToRadians(30)
              ));
        //@formatter:on

        // put(new PhotonCamera("Forward Left"), robotTip.plus(forwardLeftDelta));
        // put(new PhotonCamera("Forward Right"), robotTip.plus(forwardRightDelta));
        // put(new PhotonCamera("Backward Left"), robotTip.plus(backwardLeftDelta));
        // put(new PhotonCamera("Backward Right"), robotTip.plus(backwardRightDelta));

      }
    };

    private static AprilTagFieldLayout FIELD_ATFL;
    static {
      try {
        FIELD_ATFL = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
      } catch (Exception e) {
        FIELD_ATFL = new AprilTagFieldLayout(Collections.emptyList(), FIELD_CONSTANTS.LENGTH, FIELD_CONSTANTS.WIDTH);
      }
    }

    private static final AprilTagFieldLayout SHOP_ATFL;
    static {
      var apriltagsCopy = new ArrayList<AprilTag>();
      for (var apriltag : FIELD_ATFL.getTags()) {
        apriltagsCopy.add(new AprilTag(apriltag.ID, apriltag.pose));
      }

      apriltagsCopy.get(4 - 1).pose = new Pose3d(Units.inchesToMeters(277.4) + Units.inchesToMeters(60),
          Units.inchesToMeters(84.3) + Units.inchesToMeters(10), apriltagsCopy.get(4 - 1).pose.getZ(),
          new Rotation3d(0, 0, Units.degreesToRadians(180)));

      SHOP_ATFL = new AprilTagFieldLayout(apriltagsCopy, FIELD_CONSTANTS.LENGTH, FIELD_CONSTANTS.WIDTH);
    }

    private static final AprilTagFieldLayout TEST_ATFL;
    static {
      var id1Pose = new Pose3d(2.5, 2.5, 0, new Rotation3d(0, 0, 0));
      var id1Toid8 = new Transform3d(new Translation3d(0, 0.2159, 0), new Rotation3d(0, 0, 0));
      var id8Pose = id1Pose.plus(id1Toid8);

      var testTags = new ArrayList<AprilTag>();
      testTags.add(new AprilTag(1, id1Pose));
      testTags.add(new AprilTag(8, id8Pose));

      TEST_ATFL = new AprilTagFieldLayout(testTags, 5, 5);
    }

    public static final AprilTagFieldLayout ATFL;

    static {
      switch (FIELD_CONSTANTS.FIELD_TYPE) {
        case SHOP:
          ATFL = SHOP_ATFL;
          break;
        case TEST:
          ATFL = TEST_ATFL;
          break;
        default:
          ATFL = FIELD_ATFL;
          break;
      }
    }
  }

  public static final class VISION_SIM {
    public static final List<SimCamParams> SIM_CAM_PARAMS = new ArrayList<SimCamParams>();
    static {
      for (var camAndTrans : VISION.CAMS_AND_TRANS.entrySet()) {
        var camName = camAndTrans.getKey().getName();
        var trans = camAndTrans.getValue();
        SIM_CAM_PARAMS.add(new SimCamParams(92, 9000, 640, 400, 0, trans, camName));
      }

    }

  }

  public static class AUTO_ALIGN {
    public static class COMMUNITY {
      public static final double ARM_THRESHOLD = 20;//in degrees
      public static final double ROT_THRESHOLD = 20;
      public static final double DIST_THRESHOLD = 1;
      public static final PathConstraints CONSTRAINTS = new PathConstraints(2, 1.5);
      public static final ArrayList<Translation2d> DIAG_CORNERS;
      static {
        switch (FIELD_CONSTANTS.FIELD_TYPE) {
          case SHOP:
            DIAG_CORNERS = new ArrayList<Translation2d>() {
              {
                add(new Translation2d(0, 0));
                add(new Translation2d(3, FIELD_CONSTANTS.WIDTH));
              }
            };
            break;
          default:
            DIAG_CORNERS = new ArrayList<Translation2d>() {
              {
                add(new Translation2d(0, 0));
                add(new Translation2d(3.8, 5.56));
              }
            };
            break;
        }
      }
    }
    public static class FEEDER {
      public static final double Y_THRESHOLD = 0.5;
      public static final double ROT_THRESHOLD = 20;//in degrees
      public static final double ARM_THRESHOLD = 20;//in degrees
      public static final double WIDTH = FIELD_CONSTANTS.WIDTH - 5.6;
      public static final double LENGTH = FIELD_CONSTANTS.LENGTH - 10;
      public static final double DIST_THRESHOLD = 1;
      public static final Transform2d PREP2DEST = new Transform2d(new Translation2d(-1, 0), new Rotation2d());
      public static final PathConstraints CONSTRAINTS = new PathConstraints(2, 1.5);

      public static Pose2d BLUE_RIGHT_PREP_POSE;
      public static Pose2d BLUE_LEFT_PREP_POSE;
      public static Pose2d BLUE_RIGHT_DEST_POSE;
      public static Pose2d BLUE_LEFT_DEST_POSE;

      public static Pose2d RED_RIGHT_PREP_POSE;
      public static Pose2d RED_LEFT_PREP_POSE;
      public static Pose2d RED_RIGHT_DEST_POSE;
      public static Pose2d RED_LEFT_DEST_POSE;

      public static final ArrayList<Translation2d> DIAG_CORNERS;
      public static final double CLOSE_THRESHOLD = 3;

      static {
        switch (FIELD_CONSTANTS.FIELD_TYPE) {
          case SHOP:
            BLUE_RIGHT_DEST_POSE = new Pose2d(8.08, 3.2, Rotation2d.fromDegrees(180));
            BLUE_LEFT_DEST_POSE = new Pose2d(8.08, 3.2, Rotation2d.fromDegrees(180));
            RED_RIGHT_DEST_POSE = new Pose2d(8.08, 3.2, Rotation2d.fromDegrees(180));
            RED_LEFT_DEST_POSE = new Pose2d(8.08, 3.2, Rotation2d.fromDegrees(180));
            DIAG_CORNERS = new ArrayList<Translation2d>() {
              {
                var corner = new Translation2d(FIELD_CONSTANTS.LENGTH / 2, FIELD_CONSTANTS.WIDTH / 2);
                add(corner);
                add(new Translation2d(4, 0));
              }
            };
            break;
          default:
            BLUE_RIGHT_DEST_POSE = new Pose2d(15.772, 5.949, Rotation2d.fromDegrees(180)); // -0.02
            BLUE_LEFT_DEST_POSE = new Pose2d(15.78, 7.593, Rotation2d.fromDegrees(180));//was 15.82 in calibration
            RED_RIGHT_DEST_POSE = new Pose2d(15.769, 0.424, Rotation2d.fromDegrees(180)); // -0.02
            RED_LEFT_DEST_POSE = new Pose2d(15.728, 2.088, Rotation2d.fromDegrees(180)); // -0.02
            DIAG_CORNERS = new ArrayList<Translation2d>() {
              {
                add(new Translation2d(9.8, 5.6));
                add(new Translation2d(FIELD_CONSTANTS.LENGTH, FIELD_CONSTANTS.WIDTH));
              }
            };
            break;
        }
        BLUE_RIGHT_PREP_POSE = BLUE_RIGHT_DEST_POSE.transformBy(PREP2DEST.inverse());
        BLUE_LEFT_PREP_POSE = BLUE_LEFT_DEST_POSE.transformBy(PREP2DEST.inverse());
        RED_RIGHT_PREP_POSE = BLUE_RIGHT_DEST_POSE.transformBy(PREP2DEST.inverse());
        RED_LEFT_PREP_POSE = BLUE_LEFT_DEST_POSE.transformBy(PREP2DEST.inverse());
      }
    }
  }
}
