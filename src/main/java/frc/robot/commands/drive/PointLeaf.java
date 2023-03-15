package frc.robot.commands.drive;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PointLeaf {
  final Pose2d pose;
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

    var endNode = new PointLeaf(endPose, level2Left, level2Right);
    return endNode;
  }
}
