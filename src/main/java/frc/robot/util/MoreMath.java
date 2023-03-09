package frc.robot.util;

import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class MoreMath {

  private MoreMath() {}

  // More or less https://stackoverflow.com/a/5295202 but could be easily reimplemented
  public static double round(double num, int places) {
    return ((int) (num * Math.pow(10, places))) / (double) Math.pow(10, places);
  }

  public static String pos2dToString(Pose2d pos, int places) {
    // return "X="  + round(pos.getX(), places) + 
    //      "; Y="  + round(pos.getY(), places) + 
    //      "; Deg="+ round(pos.getRotation().getDegrees(), places);
    return "X=" + pos.getX() + "; Y=" + pos.getY() + "; Deg=" + pos.getRotation().getDegrees();
  }

  public static String cornerToString(TargetCorner corner, int places) {
    return "(" + round(corner.x, places) + ", " + round(corner.y, places) + ")";
  }

  /* Pose exp for teleop drive. Takes in controller x and y axises and outputs pose exped ones */
  public static Pair<Double, Double> poseExp(double x, double y) {
    // java doesn't have an exponent operator apparently
    double mag = Math.sqrt(x * x + y * y);
    double angle = Math.atan2(y, x);

    mag *= Math.abs(mag);

    return new Pair<>(Math.cos(angle) * mag, Math.sin(angle) * mag);
  }

  public static double getClosest(double current, double desired){
    var currentWrap = getWrap(current);
    var currentBase = currentWrap*360;

    var desiredMod360 = mod360(desired);
    
    double[] possibilities = {
        (currentBase+desiredMod360-720),
        (currentBase+desiredMod360-360),
        (currentBase+desiredMod360),
        (currentBase+desiredMod360+360),
        (currentBase+desiredMod360+720)
    };

    var closest = (double) Integer.MIN_VALUE;
    for(var possibility: possibilities){
        var currentDist = Math.abs(possibility - current);
        var closestDist = Math.abs(closest - current);
        if(closestDist>currentDist){
            closest = possibility;
        }
    }
    // System.out.println(index);
    return closest;
  }

  public static double getClosestRad(double current, double desired){
    var result = getClosest(Units.radiansToDegrees(current), Units.radiansToDegrees(desired));
    return Units.degreesToRadians(result);
  }

  
  public static int getWrap(double angle){
    var wrap = (int) (angle / 360.0);
    return wrap;
  }

  public static double mod360(double angle){
    var angleMod360 = angle%360;
    angleMod360 = angleMod360<0?angleMod360+360:angleMod360;
    return angleMod360;
  }
}
