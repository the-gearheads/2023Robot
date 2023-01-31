package frc.robot.util;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;

public class AdditionalMathUtils {

  private AdditionalMathUtils() {}

  // More or less https://stackoverflow.com/a/5295202 but could be easily reimplemented
  public static double round(double num, int places) {
    return ((int) (num * Math.pow(10, places))) / Math.pow(10, places);
  }

  public static String pos2dToString(Pose2d pos, int places) {
    return "X="  + round(pos.getX(), places) + 
         "; Y="  + round(pos.getY(), places) + 
         "; Deg="+ round(pos.getRotation().getDegrees(), places);
  }

  /* Pose exp for teleop drive. Takes in controller x and y axises and outputs pose exped ones */
  public static Pair<Double, Double> poseExp(double x, double y) {
    // java doesn't have an exponent operator apparently
    double mag = Math.sqrt(x*x + y*y);
    double angle = Math.atan2(y, x);

    mag *= Math.abs(mag);

    return new Pair<>(Math.cos(angle) * mag, Math.sin(angle) * mag);
  }
}
