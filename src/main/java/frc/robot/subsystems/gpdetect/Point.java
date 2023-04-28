package frc.robot.subsystems.gpdetect;

public final class Point {
  public final double x;
  public final double y;

  public Point() {
    this.x = 0;
    this.y = 0;
  }

  public Point(double x, double y) {
    this.x = x;
    this.y = y;
  }

  @Override
  public String toString() {
    return "(PT) X: " + x + ", Y: " + y;
  }
}
