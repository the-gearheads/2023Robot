package frc.robot.util.multicamvision;

import java.util.Arrays;
import java.util.Objects;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Represents an odometry record. The record contains the inputs provided as well as the pose that was observed based on these inputs, as well as the previous record and
 * its inputs.
 */
public class InterpolationRecord implements Interpolatable<InterpolationRecord> {
  // The pose observed given the current sensor inputs and the previous pose.
  public final Pose2d poseMeters;

  // The current gyro angle.
  public final Rotation2d gyroAngle;

  // The distances and rotations measured at each module.
  private final SwerveModulePosition[] modulePositions;
  private final int m_numModules = 4;

  private SwerveDriveKinematics m_kinematics;

  /**
   * Constructs an Interpolation Record with the specified parameters.
   *
   * @param pose The pose observed given the current sensor inputs and the previous pose.
   * @param gyro The current gyro angle.
   * @param wheelPositions The distances and rotations measured at each wheel.
   */
  public InterpolationRecord(Pose2d poseMeters, Rotation2d gyro, SwerveModulePosition[] modulePositions,
      SwerveDriveKinematics kinematics) {
    this.poseMeters = poseMeters;
    this.gyroAngle = gyro;
    this.modulePositions = modulePositions;
    this.m_kinematics = kinematics;
  }

  /**
   * Return the interpolated record. This object is assumed to be the starting position, or lower bound.
   *
   * @param endValue The upper bound, or end.
   * @param t How far between the lower and upper bound we are. This should be bounded in [0, 1].
   * @return The interpolated value.
   */
  @Override
  public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
    if (t < 0) {
      return this;
    } else if (t >= 1) {
      return endValue;
    } else {
      // Find the new wheel distances.
      var modulePositions = new SwerveModulePosition[m_numModules];

      // Find the distance travelled between this measurement and the interpolated measurement.
      var moduleDeltas = new SwerveModulePosition[m_numModules];

      for (int i = 0; i < m_numModules; i++) {
        double ds =
            MathUtil.interpolate(this.modulePositions[i].distanceMeters, endValue.modulePositions[i].distanceMeters, t);
        Rotation2d theta = this.modulePositions[i].angle.interpolate(endValue.modulePositions[i].angle, t);
        modulePositions[i] = new SwerveModulePosition(ds, theta);
        moduleDeltas[i] = new SwerveModulePosition(ds - this.modulePositions[i].distanceMeters, theta);
      }

      // Find the new gyro angle.
      var gyro_lerp = gyroAngle.interpolate(endValue.gyroAngle, t);

      // Create a twist to represent this change based on the interpolated sensor inputs.
      Twist2d twist = m_kinematics.toTwist2d(moduleDeltas);
      twist.dtheta = gyro_lerp.minus(gyroAngle).getRadians();

      return new InterpolationRecord(poseMeters.exp(twist), gyro_lerp, modulePositions, m_kinematics);
    }
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!(obj instanceof InterpolationRecord)) {
      return false;
    }
    InterpolationRecord record = (InterpolationRecord) obj;
    return Objects.equals(gyroAngle, record.gyroAngle) && Arrays.equals(modulePositions, record.modulePositions)
        && Objects.equals(poseMeters, record.poseMeters);
  }

  @Override
  public int hashCode() {
    return Objects.hash(gyroAngle, Arrays.hashCode(modulePositions), poseMeters);
  }
}
