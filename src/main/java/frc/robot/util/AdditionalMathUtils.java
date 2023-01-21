package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class AdditionalMathUtils {
  // More or less https://stackoverflow.com/a/5295202 but could be easily reimplemented
  public static double round(double num, int places){
    return ((int)(num*Math.pow(10, places))) / Math.pow(10, places);
  }
  public static String pos2dToString(Pose2d pos, int places){
    return "X="+round(pos.getX(),places)+"; Y="+round(pos.getY(),places) + "; Deg=" + round(pos.getRotation().getDegrees(),places);
  }
}
