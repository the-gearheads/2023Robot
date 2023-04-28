package frc.robot.subsystems.gpdetect;

import java.util.ArrayList;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.gpdetect.Detection.GamePiece;

public class GamePieceDetector extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("/GPDetect");
  private int packetLength = 6;
  private ArrayList<Detection> detections = new ArrayList<>(packetLength * 10);
  private DoubleArraySubscriber sub = table.getDoubleArrayTopic("Detections").subscribe(new double[] {});

  @Override
  public void periodic() {
    // intended format: type(0==cube, 1==cone), confidence, 4x corners (x, then y, origin top left)
    var data = sub.get();
    if (data.length < packetLength) {
      return;
    }

    detections.clear();
    for (int i = 0; i < data.length % packetLength; i++) {
      GamePiece gp = GamePiece.fromInt((int) data[i * packetLength + 0]);
      double confidence = data[i * packetLength + 1];
      ArrayList<Point> points = new ArrayList<>(4);
      for (int j = 0; j < 4; j++) {
        points.add(new Point(data[i * packetLength + 2 * j + 0], data[i * packetLength + 2 * j + 1]));
      }
      detections.add(new Detection(gp, confidence, points.toArray(new Point[points.size()])));
    }

  }
}
