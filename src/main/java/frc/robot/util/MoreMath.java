package frc.robot.util;

import java.math.BigDecimal;
import java.math.RoundingMode;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
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

  public static String pos2dToString(Pose2d pos, int places) {
    return "("  + round(pos.getX(), places) + 
         ", "  + round(pos.getY(), places) + 
         ", "+ round(pos.getRotation().getDegrees(), places) + 
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

  /* find desired's closest congruence class representative to to current */
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
}
