package frc.robot.util;

import java.math.BigDecimal;
import java.math.RoundingMode;
import org.photonvision.targeting.TargetCorner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class MoreMath {
  
  public static double round(double value, int places) {
    if (places < 0) throw new IllegalArgumentException();
    if (Double.isNaN(value)) return value;
    if (Double.isInfinite(value)) return value;

      BigDecimal bd = new BigDecimal(Double.toString(value));
      bd = bd.setScale(places, RoundingMode.HALF_UP);
      return bd.doubleValue();
}

  public static String pose2dToString(Pose2d pos, int places) {
    return "("  + round(pos.getX(), places) + 
         ", "  + round(pos.getY(), places) + 
         ", "+ round(pos.getRotation().getDegrees(), places) + 
         ")";
    // return "X=" + pos.getX() + "; Y=" + pos.getY() + "; Deg=" + pos.getRotation().getDegrees();
  }

  
  public static String pose3dToString(Pose3d pose, int places) {
    return "("  + round(pose.getX(), places) + 
         ", "  + round(pose.getY(), places) + 
         ", "  + round(pose.getZ(), places) +
         ";" + 
         " "+ round(Units.radiansToDegrees(pose.getRotation().getX()), places) + 
         ", "+ round(Units.radiansToDegrees(pose.getRotation().getY()), places) + 
         ", "+ round(Units.radiansToDegrees(pose.getRotation().getZ()), places) + 
         ")";
    // return "X=" + pos.getX() + "; Y=" + pos.getY() + "; Deg=" + pos.getRotation().getDegrees();
  }

  public static String transform3dToString(Transform3d transform, int places) {
    return "("  + round(transform.getX(), places) + 
         ", "  + round(transform.getY(), places) + 
         ", "  + round(transform.getZ(), places) +
         ";" + 
         " "+ round(Units.radiansToDegrees(transform.getRotation().getX()), places) + 
         ", "+ round(Units.radiansToDegrees(transform.getRotation().getY()), places) + 
         ", "+ round(Units.radiansToDegrees(transform.getRotation().getZ()), places) + 
         ")";
    // return "X=" + pos.getX() + "; Y=" + pos.getY() + "; Deg=" + pos.getRotation().getDegrees();
  }

  public static String cornerToString(TargetCorner corner, int places){
    return "("  + round(corner.x, places) + 
         ", "  + round(corner.y, places)+
         ")";
  }

  /* Pose exp for teleop drive. Takes in controller x and y axises and outputs pose exped ones */
  public static Pair<Double, Double> poseExp(double x, double y) {
    // java doesn't have an exponent operator apparently
    double mag = Math.sqrt(x * x + y * y);
    double angle = Math.atan2(y, x);

    mag *= Math.abs(mag);

    return new Pair<>(Math.cos(angle) * mag, Math.sin(angle) * mag);
  }

  public static double getClosest(double current, double desired) {
    var currentWrap = getWrap(current);
    var currentBase = currentWrap * 360;

    var desiredMod360 = mod360(desired);

    double[] possibilities = {(currentBase + desiredMod360 - 720), (currentBase + desiredMod360 - 360),
        (currentBase + desiredMod360), (currentBase + desiredMod360 + 360), (currentBase + desiredMod360 + 720)};

    var closest = (double) Integer.MIN_VALUE;
    for (var possibility : possibilities) {
      var currentDist = Math.abs(possibility - current);
      var closestDist = Math.abs(closest - current);
      if (closestDist > currentDist) {
        closest = possibility;
      }
    }
    // System.out.println(index);
    return closest;
  }

  /* find desired's closest congruence class representative to current */
  public static double getClosestRad(double current, double desired) {
    var result = getClosest(Units.radiansToDegrees(current), Units.radiansToDegrees(desired));
    return Units.degreesToRadians(result);
  }


  public static int getWrap(double angle) {
    var wrap = (int) (angle / 360.0);
    return wrap;
  }

  public static double mod360(double angle) {
    var angleMod360 = angle % 360;
    angleMod360 = angleMod360 < 0 ? angleMod360 + 360 : angleMod360;
    return angleMod360;
  }

  public static Pose2d deepCopyPose(Pose2d pose) {
    var deepCopy = new Pose2d();
    deepCopy = deepCopy.plus(new Transform2d(new Pose2d(), pose));
    return deepCopy;
  }

  public static Pose2d transformByAlliance(Pose2d pose) {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      return new Pose2d(pose.getX(), Constants.FIELD_CONSTANTS.WIDTH - pose.getY(), pose.getRotation());
    }
    return pose;
  }

  private static double calculateDifference(Pose3d x, Pose3d y) {
    return x.getTranslation().getDistance(y.getTranslation());
  }

  /* horizFOV and verFOV expected in degrees */
  public static Matrix<N3, N3> calcCamMatrix(double resWidth, double resHeight, double horizFOV, double verFOV) {
    horizFOV = Units.degreesToRadians(horizFOV);
    verFOV = Units.degreesToRadians(verFOV);


    var f_x = resWidth / (Math.tan(horizFOV / 2) * 2);
    var f_y = resHeight / (Math.tan(verFOV / 2) * 2);

    var camMatrix = new Matrix<>(Nat.N3(), Nat.N3());
    camMatrix.setColumn(0, VecBuilder.fill(f_x, 0, 0));
    camMatrix.setColumn(1, VecBuilder.fill(0, f_y, 0));
    camMatrix.setColumn(2, VecBuilder.fill(resWidth / 2, resHeight / 2, 1));

    return camMatrix;
  }

  public static Pose3d transformBy(Pose3d current, Transform3d other){
    return new Pose3d(
      current.getTranslation().plus(other.getTranslation().rotateBy(current.getRotation())),
      addRotation3d(current.getRotation(), other.getRotation()));
  }

  public static Pose3d transformTwice(Pose3d current, Transform3d other, Transform3d another){
    return transformBy(transformBy(current, other), another);
  }
  
  public static Rotation3d addRotation3d(Rotation3d current, Rotation3d other){
    return new Rotation3d(
      current.getX() + other.getX(),
      current.getY() + other.getY(),
      current.getZ() + other.getZ()
    );
  }

  public static Rotation2d calcHeading(Pose2d start, Pose2d end){
    var heading = end.getTranslation().minus(start.getTranslation()).getAngle();
    return heading;
  }

  public static PathPlannerTrajectory createStraightPath(Pose2d start, Pose2d end, PathConstraints constraints){
      var heading = calcHeading(start, end);      
      var startPoint = createPathPoint(start, heading);
      var endPoint = createPathPoint(end, heading);

      var traj = PathPlanner.generatePath(constraints, startPoint, endPoint);
      return traj;
  }

  public static PathPoint createPathPoint(Pose2d pose, Rotation2d heading){
    return new PathPoint(pose.getTranslation(), heading, pose.getRotation());
  }

  public static boolean within(double val, double firstBound, double secondBound){
      double lowerBound;
      double upperBound;
      if(firstBound<secondBound){
        lowerBound = firstBound;
        upperBound = secondBound;
      }else{
        lowerBound = secondBound;
        upperBound = firstBound;
      }

      return val > lowerBound && val < upperBound;
  }

  public static boolean isBlue(){
    return DriverStation.getAlliance() == DriverStation.Alliance.Blue;
  }
}
