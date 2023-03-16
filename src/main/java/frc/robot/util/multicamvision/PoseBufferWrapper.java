package frc.robot.util.multicamvision;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;

public class PoseBufferWrapper {
  private SwerveDriveKinematics kinematics;
  private Supplier<SwerveModulePosition[]> modulePoseLambda;
  private Supplier<Rotation2d> gyroLambda;
  private Supplier<Pose2d> poseLambda;
  private TimeInterpolatableBuffer<InterpolationRecord> poseBuffer;
  private static PoseBufferWrapper instance;

  public PoseBufferWrapper(Supplier<Pose2d> poseLambda, Supplier<SwerveModulePosition[]> modulePoseLambda,
      Supplier<Rotation2d> gyroLambda, SwerveDriveKinematics kinematics, Consumer<Runnable> setResetBuffer) {
    this.poseLambda = poseLambda;
    this.gyroLambda = gyroLambda;
    this.modulePoseLambda = modulePoseLambda;
    this.kinematics = kinematics;
    this.poseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);
    setResetBuffer.accept(this::reset);

    instance = this;
  }

  public static PoseBufferWrapper getInstance(){
    return instance;
  }

  public void update() {
    var pose = poseLambda.get();
    var modulePoses = modulePoseLambda.get();
    var gyro = gyroLambda.get();

    poseBuffer.addSample(Timer.getFPGATimestamp(), new InterpolationRecord(pose, gyro, modulePoses, kinematics));
  }

  public void reset() {
    poseBuffer.clear();
  }

  public Optional<InterpolationRecord> getSample(double timestamp) {
    return poseBuffer.getSample(timestamp);
  }
}
