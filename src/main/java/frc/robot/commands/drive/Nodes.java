// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.MoreMath;

/** Add your docs here. */
public class Nodes {
  public static enum NodeY {
    // format: off
    // no bump -> bump
    N1(1, 4.93), N2(2, 4.41), N3(3, 3.85), N4(4, 3.32), N5(5, 2.76), N6(6, 2.175), N7(7, 1.63), N8(8, 1.083), N9(9,
        0.544);
    // format: on

    private final double y;
    public final int index;

    NodeY(int index, double y) {
      this.index = index;
      this.y = y;
    }

    public static NodeY getByIndex(int index) {
      for (var node : NodeY.values()) {
        if (node.index == index) {
          return node;
        }
      }
      return null;
    }

    public double getY() {
      var transformedY = MoreMath.transformByAlliance(new Pose2d(0, this.y, new Rotation2d())).getY();
      return transformedY;
    }
  }
  public static enum NodeX {
    LOW(1, 2.15), MID(2, 2.05), HIGH(3, 1.9);

    public final double x;
    public final int index;

    NodeX(int index, double x) {
      this.index = index;
      this.x = x;
    }

    public static NodeX getByIndex(int index) {
      for (var node : NodeX.values()) {
        if (node.index == index) {
          return node;
        }
      }
      return null;
    }
  }
}
