package frc.robot.commands.drive;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PointLeaf {
    public enum NodeY{
      N1(1,4.93), N2(2,4.41), N3(3,3.85), 
      N4(4,3.32), N5(5,2.76), N6(6,2.175),
      N7(7,1.63), N8(8,1.083), N9(9,0.544);
      
      public final double y;
      public final int index;
  
      NodeY(int index, double y){
        this.index=index;
        this.y=y;
      }

      public static NodeY getByIndex(int index){
        for(var node : NodeY.values()){
          if(node.index==index){
            return node;
          }
        }
        return null;
      }
    }
    public enum NodeX{
      LOW(1, 2.15), MID(2, 2.05), HIGH(3, 1.9);

      public final double x;
      public final int index;

      NodeX(int index, double x){
        this.index=index;
        this.x=x;
      }
      public static NodeX getByIndex(int index){
        for(var node : NodeX.values()){
          if(node.index==index){
            return node;
          }
        }
        return null;
      }
    }
  

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

    var level3 = new PointLeaf(new Pose2d(2.3, endPose.getY(), rot), level2Left, level2Right);
    var endNode = new PointLeaf(endPose, level3);
    return endNode;
  }
}
