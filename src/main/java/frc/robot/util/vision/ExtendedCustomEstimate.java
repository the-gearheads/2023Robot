package frc.robot.util.vision;

import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class ExtendedCustomEstimate extends CustomEstimate {
  public final Pose3d alt;

  public ExtendedCustomEstimate(Pose3d best, Pose3d alt, double ambiguity, double timestampSeconds,
      List<PhotonTrackedTarget> targetsUsed, Matrix<N3, N1> confidence) {
    super(best, ambiguity, timestampSeconds, targetsUsed, confidence);
    this.alt = alt;
  }

  public ExtendedCustomEstimate(Pose3d best, Pose3d alt, double ambiguity, double timestampSeconds,
      List<PhotonTrackedTarget> targetsUsed) {
    super(best, ambiguity, timestampSeconds, targetsUsed);
    this.alt = alt;
  }

  public CustomEstimate getBestEstimate() {
    return new CustomEstimate(best, ambiguity, timestampSeconds, targetsUsed);
  }

  public CustomEstimate getAltEstimate() {
    return new CustomEstimate(alt, ambiguity, timestampSeconds, targetsUsed);
  }
}
