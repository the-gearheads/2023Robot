package frc.robot.subsystems.gpdetect;

import java.util.EnumSet;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GamePieceDetector extends SubsystemBase {
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public int detectionListener = inst.addListener(inst.getEntry("/GpDetect/Detections"), EnumSet.of(Kind.kValueAll), this::onDetectionUpdate);

  public void onDetectionUpdate(NetworkTableEvent event) {
    // intended format: type(0==cube, 1==cone), confidence, corners (x, then y, origin top left)
    var data = event.valueData.value.getStringArray();
  }
}
