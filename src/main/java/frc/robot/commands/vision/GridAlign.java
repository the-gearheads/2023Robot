package frc.robot.commands.vision;

import java.util.ArrayList;
import org.photonvision.PhotonPoseEstimator;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Swerve;

public class GridAlign extends CommandBase {
  private PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout atfl;
  private Swerve swerveSubsystem;
  private NodePlace nodePlace;
  private Pose2d destPose;

  // differences in location from the april tags to the center, right, and left nodes
  public enum NodePlace {
    RIGHT(new Translation2d(-2, 1)), CENTER(new Translation2d(0, 1)), LEFT(new Translation2d(2, 1));

    private Translation2d val;

    private NodePlace(Translation2d val) {
      this.val = val;
    }
  }

  public GridAlign(Vision vision, Swerve swerve, NodePlace nodePlace) {
    this.atfl = vision.getAtfl();
    this.swerveSubsystem = swerve;
    this.nodePlace = nodePlace;
  }

  public void initialize() {
    Pose2d robotPos = this.swerveSubsystem.getPose();
    double minDistance = 1e10;
    Pose2d closestIdPose = new Pose2d();
    ArrayList<Integer> aprilTagIds;

    // create the lists of the node Apriltags depending on the alliance
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      aprilTagIds = new ArrayList<Integer>() {
        {
          add(1);
          add(2);
          add(3);
        }
      };
    } else {
      aprilTagIds = new ArrayList<Integer>() {
        {
          add(6);
          add(7);
          add(8);
        }
      };
    }

    // gets the closest node april tag 
    for (int i : aprilTagIds) {
      Pose2d tagPose = this.atfl.getTagPose(i).get().toPose2d();
      double distance = tagPose.getTranslation().getDistance(robotPos.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestIdPose = tagPose;
      }
    }

    // goto closest apriltag + the offset from apriltag to node
    this.destPose = closestIdPose.plus(new Transform2d(nodePlace.val, Rotation2d.fromDegrees(180)));
  }

  public void end() {
    swerveSubsystem.goTo(destPose, Constants.AUTON.SLOW_CONSTRAINTS).schedule();
  }

  public boolean isFinished() {
    return true;
  }
}
