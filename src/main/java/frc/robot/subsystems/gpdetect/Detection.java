package frc.robot.subsystems.gpdetect;

public class Detection {
  public enum GamePiece {
    CONE, CUBE;

    public static GamePiece fromInt(int num) {
      return num == 0 ? CONE : CUBE;
    }
  }

  public final Point topLeft;
  public final Point bottomRight;
  public final Point centerPoint;
  public final double confidence;
  public final double area;
  public final GamePiece type;

  public Detection(GamePiece type, double confidence, Point[] corners) {
    this.type = type;
    this.confidence = confidence;
    this.topLeft = corners[0];
    this.bottomRight = corners[1];
    this.area = calculateArea();
    this.centerPoint = calculateCenterPoint();
  }

  private Point calculateCenterPoint() {
    return new Point((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);
  }

  private double calculateArea() {
    return (bottomRight.x - topLeft.x) * (bottomRight.y - topLeft.y);
  }

  @Override
  public String toString() {
    // spotless: off
    return new StringBuilder()
        .append("(DET) T: ").append(type.name())
        .append(", C: ").append(Double.toString(confidence))
        .append(", C-P: ").append(centerPoint.toString())
        .append(", TL: ").append(topLeft.toString())
        .append(", BR: ").append(bottomRight.toString()).toString();
    // spotless: on
  }
}
