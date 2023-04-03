package frc.robot.commands.drive.obsolete;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.MoreMath;

public class PointLeaf {
  Pose2d pose;
  final List<PointLeaf> children;

  public PointLeaf(Pose2d pose, PointLeaf... children) {
    this.pose = pose;
    this.children = Arrays.asList(children);
  }

  public PointLeaf(Pose2d pose, PointLeaf child) {
    this.pose = pose;
    this.children = Collections.singletonList(child);
  }

  public PointLeaf(Pose2d pose) {
    this.pose = pose;
    this.children = Collections.emptyList();
  }

  public double getDistance(PointLeaf other) {
    return this.pose.getTranslation().getDistance(other.pose.getTranslation());
  }

  public void transformByAlliance() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
      return;

    this.pose = MoreMath.transformByAlliance(this.pose);
    for (var child : children) {
      child.transformByAlliance();
    }
  }

  public PointLeaf getClosestChild(PointLeaf other) {
    PointLeaf closestChild = null;
    var closestDistance = 1000d;

    for (PointLeaf child : children) {
      var currentDistance = child.getDistance(other);

      if (currentDistance < closestDistance) {
        closestChild = child;
        closestDistance = currentDistance;
      }
    }

    return closestChild;
  }

  public static PointLeaf createTree(Pose2d endPose) {
    var rot = Rotation2d.fromDegrees(180);

    var level1Left = new PointLeaf(new Pose2d(5.5, 4.7, rot));
    var level1Right = new PointLeaf(new Pose2d(5.5, 0.8, rot));

    var level2Left = new PointLeaf(new Pose2d(2.3, 4.67, rot), level1Left);
    var level2Right = new PointLeaf(new Pose2d(2.3, 0.8, rot), level1Right);

    var level3 = new PointLeaf(new Pose2d(2.3, endPose.getY(), rot), level2Left, level2Right);
    var endNode = new PointLeaf(endPose, level3);

    endNode.transformByAlliance();
    return endNode;
  }
}
