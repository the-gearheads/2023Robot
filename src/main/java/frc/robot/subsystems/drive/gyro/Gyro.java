package frc.robot.subsystems.drive.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyro extends AHRS implements GyroIO {
  private boolean isCounterClockwise;

  public Gyro(SPI.Port port) {
    this(port, false);
  }

  public Gyro(SPI.Port port, boolean isCounterClockwise) {
    super(port);
    this.isCounterClockwise = isCounterClockwise;
  }

  public void setCounterClockwise(boolean isInverted) {
    this.isCounterClockwise = isInverted;
  }

  @Override
  public Rotation2d getRotation2d() {
    double direction = isCounterClockwise ? 1 : -1;//This one is different because getRotation2d already negates getAngle()!!!!
    return new Rotation2d(direction * super.getRotation2d().getRadians());
  }

  public void setRotation2d(Rotation2d newRotation2d) {
    double direction = isCounterClockwise ? -1 : 1;
    double newAngle = newRotation2d.getDegrees() * direction;
    super.zeroYaw();
    super.setAngleAdjustment(newAngle);
  }

  @Override
  public double getRate() {
    double direction = isCounterClockwise ? -1 : 1;
    return direction * super.getRate();
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.angleRadians = getRotation2d().getRadians();
    inputs.angleRate = getRate();
  }
}
