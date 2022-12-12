// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
  public static final class Drivetrain {
    public static int FL_DRIVE_ID = 27;
    public static int FL_STEER_ID = 34;
    public static int FR_DRIVE_ID = 15;
    public static int FR_STEER_ID = 45;
    public static int RL_DRIVE_ID = 6;
    public static int RL_STEER_ID = 49;
    public static int RR_DRIVE_ID = 5;
    public static int RR_STEER_ID = 50;

    /* Units per rotation for analog/absolute encoders on Talon SRX */
    public static double ANALOG_UPR = 1024;

    public static double WHEEL_DIAMETER = (4 * 2.54) / 100; // Convert to meters
    public static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static double DRIVE_GEAR_RATIO = 6.67/1;
    public static double STEER_GEAR_RATIO = 1;

    public static double STEER_F = 0.0;
    public static double STEER_P = 12.5;
    public static double STEER_I = 0.0;
    public static double STEER_D = 0.0;

    public static double DRIVE_KS = 0.18;
    public static double DRIVE_KV = 2.7;
    public static double DRIVE_KA = 0.35632;
    public static Pose2d zeroPos = new Pose2d(0,0,new Rotation2d(0));
    public static Transform2d zeroTransform = new Transform2d(new Translation2d(0,0), new Rotation2d(0));

    public static Translation2d FL_POS = new Translation2d(0.3, 0.55/2);
    public static Translation2d FR_POS = new Translation2d(0.3, -0.55/2);
    public static Translation2d RL_POS = new Translation2d(-0.3, 0.55/2);
    public static Translation2d RR_POS = new Translation2d(-0.3, -0.55/2);

    public static Rotation2d FL_OFFSET = Rotation2d.fromDegrees((611.718+362)%360); 
    public static Rotation2d FR_OFFSET = Rotation2d.fromDegrees((612.77+355)%360);
    public static Rotation2d RL_OFFSET = Rotation2d.fromDegrees((472.85)%360);
    public static Rotation2d RR_OFFSET = Rotation2d.fromDegrees((684.5+554+367)%360);

    public static double MAX_LIN_VEL = 5;//set to 2
    public static double MAX_ROT_VEL = 5;

    public static final class Auton {
      public static PIDController X_PID = new PIDController(0, 0, 0);
      public static PIDController Y_PID = new PIDController(0, 0, 0);
      public static PIDController ROT_PID = new PIDController(0, 0, 0);
      public static PathConstraints CONSTRAINTS = new PathConstraints(1, 1.5);
    }

    public static final class Sim {
      public static final class CIMSteer {
        public static double kV = 1;
        public static double kA = 1;
        public static DCMotor motor = DCMotor.getCIM(1);
      }

      public static final class NEODrive {
        public static double kV = 1;
        public static double kA = 1;
        public static DCMotor motor = DCMotor.getNEO(1);
      }
    }
  }

  public static final class Controllers {
    public static double JOYSTICK_DEADBAND = 0.05;
  }

}
