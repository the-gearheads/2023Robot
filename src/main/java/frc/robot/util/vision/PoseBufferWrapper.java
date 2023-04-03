package frc.robot.util.vision;

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
  private static PoseBufferWrapper wheelBufferInstance;
  private static PoseBufferWrapper poseBufferInstance;

  private PoseBufferWrapper(Supplier<Pose2d> poseLambda, Supplier<SwerveModulePosition[]> modulePoseLambda,
      Supplier<Rotation2d> gyroLambda, SwerveDriveKinematics kinematics) {
    this.poseLambda = poseLambda;
    this.gyroLambda = gyroLambda;
    this.modulePoseLambda = modulePoseLambda;
    this.kinematics = kinematics;
    this.poseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);
  }

  public static void createBuffers(Supplier<Pose2d> poseLambda, Supplier<Pose2d> wheelPoseLambda,
      Supplier<SwerveModulePosition[]> modulePoseLambda, Supplier<Rotation2d> gyroLambda,
      SwerveDriveKinematics kinematics, Consumer<Runnable> setResetBuffer) {
    PoseBufferWrapper.wheelBufferInstance =
        new PoseBufferWrapper(wheelPoseLambda, modulePoseLambda, gyroLambda, kinematics);
    PoseBufferWrapper.poseBufferInstance = new PoseBufferWrapper(poseLambda, modulePoseLambda, gyroLambda, kinematics);
    setResetBuffer.accept(() -> {
      PoseBufferWrapper.wheelBufferInstance.reset();
      PoseBufferWrapper.poseBufferInstance.reset();
    });
  }

  public static PoseBufferWrapper getPoseInstance() {
    return PoseBufferWrapper.poseBufferInstance;
  }

  public static PoseBufferWrapper getWheelInstance() {
    return PoseBufferWrapper.wheelBufferInstance;
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
