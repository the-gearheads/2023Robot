package frc.robot.util;

public class MathUtils {
  // More or less https://stackoverflow.com/a/5295202 but could be easily reimplemented
  public static double scale(double xMin, double xMax, double yMin, double yMax, double x) {
    return ((yMax - yMin) * (x - xMin)) / (xMax - xMin);
  }
}
