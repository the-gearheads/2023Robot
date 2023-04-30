package frc.robot.subsystems.gpdetect;

import java.util.ArrayList;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gpdetect.Detection.GamePiece;

public class GamePieceDetector extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("/GPDetect");
  private DoubleArraySubscriber sub = table.getDoubleArrayTopic("Detections").subscribe(new double[] {});
  private BooleanPublisher enablePub = table.getBooleanTopic("Enabled").publish();
  private int packetLength = 6;
  private ArrayList<Detection> detections = new ArrayList<>(packetLength * 10);
  private boolean enabled = false;

  public GamePieceDetector() {
    disableDetection();
  }

  @Override
  public void periodic() {
    // intended format: type(0==cube, 1==cone), confidence, TLX, TLY, BRX, BRY
    var data = sub.get();
    if (data.length < packetLength) {
      return;
    }

    detections.clear();
    for (int i = 0; i < Math.floor(data.length / packetLength); i++) {
      GamePiece gp = GamePiece.fromInt((int) data[i * packetLength + 0]);
      double confidence = data[i * packetLength + 1];
      ArrayList<Point> points = new ArrayList<>(2);
      for (int j = 0; j < 2; j++) {
        points.add(new Point(data[i * packetLength + 2 + 2*j + 0], data[i * packetLength + 2 + 2*j + 1]));
      }
      detections.add(new Detection(gp, confidence, points.toArray(new Point[points.size()])));
    }
  }

  public Detection getDetectionByArea() {
    Detection maxArea = null;
    for(var det: detections) {
      if(maxArea == null) {maxArea = det; continue;}
      if(maxArea.area < det.area) {
        maxArea = det;
      }
    }
    return maxArea;
  }

  /* Might not be needed, evaluate later. */
  public Detection getDetectionClosestToCenter() {
    Detection closest = null;
    Point center = new Point(0.5, 0.5);
    for(var det: detections) {
      if(closest == null) {closest = det; continue;}
      if(closest.centerPoint.distanceTo(center) > det.centerPoint.distanceTo(center)) {
        closest = det;
      }
    }
    return closest;
  }

  public int getNumTargets() {
    return detections.size();
  }

  public void enableDetection() {
    enablePub.accept(true);
    enabled = true;
  }

  public void disableDetection() {
    enablePub.accept(false);
    enabled = false;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public boolean isConnected() {
    var connections = NetworkTableInstance.getDefault().getConnections();
    for (var conn : connections) {
      if(conn.remote_id.contains("GPDetect")) { 
        return true;
      }
    }
    return false;
  }
}
