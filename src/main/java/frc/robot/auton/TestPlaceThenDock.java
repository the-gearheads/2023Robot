// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.util.MoreMath;

public class TestPlaceThenDock extends ProxyCommand {
  /** Creates a new TestPlaceThenDock. */
  public TestPlaceThenDock(Swerve s) {
    super(() -> {

      var defaultPath = AutonHelper.getPathByName("StartN4-PrepareDock", Constants.AUTON.DOCK_CONSTRAINTS);
      // defaultPath = PathPlannerTrajectory.transformTrajectoryForAlliance(defaultPath, DriverStation.getAlliance());
      var startPose = defaultPath.getInitialPose();
      var endPose = MoreMath.deepCopyPose(startPose);

      var x = SmartDashboard.getNumber("DOCK TESTING/test x", 0);
      var vel = SmartDashboard.getNumber("DOCK TESTING/test vel", 0);
      var acc = SmartDashboard.getNumber("DOCK TESTING/test acc", 0);

      var translation = new Translation2d(x, 0);
      endPose = new Pose2d(endPose.getTranslation().plus(translation), endPose.getRotation());
      var end = endPose;

      var constraints = new PathConstraints(vel, acc);

      return new InstantCommand(() -> {
        s.setPose(startPose);
      }, s).andThen(new ProxyCommand(() -> {
        return s.goTo(end, constraints);
      }));
    });
  }

  public static void initDockTestingTelemetry() {
    var defaultVel = Constants.AUTON.DOCK_CONSTRAINTS.maxVelocity;
    var defaultAcc = Constants.AUTON.DOCK_CONSTRAINTS.maxAcceleration;

    var path = AutonHelper.getPathByName("StartN4-PrepareDock", Constants.AUTON.DOCK_CONSTRAINTS);

    var startPose = path.getInitialPose();
    var endPose = path.getEndState().poseMeters;
    var transform = new Transform2d(startPose, endPose);

    var translation = transform.getTranslation();

    var defaultX = translation.getX();

    SmartDashboard.putNumber("DOCK TESTING/default x", defaultX);
    SmartDashboard.putNumber("DOCK TESTING/default vel", defaultVel);
    SmartDashboard.putNumber("DOCK TESTING/default acc", defaultAcc);

    SmartDashboard.putNumber("DOCK TESTING/test x", defaultX);
    SmartDashboard.putNumber("DOCK TESTING/test vel", defaultVel);
    SmartDashboard.putNumber("DOCK TESTING/test acc", defaultAcc);
  }
}
