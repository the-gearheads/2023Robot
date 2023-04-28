package frc.robot.subsystems.gpdetect;

public class Detection {
  public enum GamePiece {
    CONE, CUBE;

    public static GamePiece fromInt(int num) {
      return num == 0 ? CONE : CUBE;
    }
  }

  public final Point[] corners;
  public final Point centerPoint;
  public final double confidence;
  public final double area;
  public final GamePiece type;
  
  public Detection(GamePiece type, double confidence, Point[] corners) {
    this.type = type;
    this.confidence = confidence;
    this.corners = corners;
    this.area = calculateArea();
    this.centerPoint = calculateCenterPoint();
  }

  /* Haven't written the coproc code yet so i make 0 assumptions about corner order and instead sort them myself */
  private double getMinOrMaxCoord(boolean getMin, boolean useXCoord) {
    double tally = -99;
    for(var point: corners) {
      double val = point.y;
      if(useXCoord) val = point.x;
      if(tally == -99) tally = val;
      tally = getMin ? Math.min(tally, val) : Math.max(tally, val);
    }
    return tally;
  }

  private Point getCorner(boolean minX, boolean minY) {
    return new Point(getMinOrMaxCoord(minX, true), getMinOrMaxCoord(minY, false));
  }

  private Point calculateCenterPoint() {
    Point bottomLeft = getCorner(true, false);
    Point bottomRight = getCorner(false, false);
    Point topRight = getCorner(false, true);

    return new Point(
      bottomLeft.x + ((bottomRight.x - bottomLeft.x) / 2),
      topRight.y + ((bottomRight.y - topRight.y) / 2)
    );
  }

  private double calculateArea() {
    Point bottomLeft = getCorner(true, false);
    Point bottomRight = getCorner(false, false);
    Point topRight = getCorner(false, true);
    return (bottomRight.x - bottomLeft.x) * (bottomRight.y - topRight.y);
  }

  @Override
  public String toString() {
    return new StringBuilder()
      .append("(DET) T: ")
      .append(type.name())
      .append(", C: ")
      .append(Double.toString(confidence))
      .append(", C-P: ")
      .append(centerPoint.toString())
      .toString();
  }
}