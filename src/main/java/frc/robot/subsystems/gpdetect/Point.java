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

  public double distanceTo(Point p2) {
    double dx = p2.x - x;
    double dy = p2.y - y;
    return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
  }

  @Override
  public String toString() {
    return "(PT) X: " + x + ", Y: " + y;
  }
}
