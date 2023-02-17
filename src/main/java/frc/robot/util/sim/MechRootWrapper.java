package frc.robot.util.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class MechRootWrapper {
  private MechanismRoot2d root;
  private double y;
  private double x;
  private Mechanism2d mech;

  public MechRootWrapper(Mechanism2d mech, String name, double x, double y) {
    this.root = mech.getRoot(name, x, y);
    this.x = x;
    this.y = y;
    this.mech = mech;
  }

  public void setPosition(double x, double y) {
    this.root.setPosition(x, y);
    this.x = x;
    this.y = y;
  }

  public Pose2d getPosition() {
    return new Pose2d(x, y, new Rotation2d());
  }

  public MechanismLigament2d append(MechanismLigament2d lig) {
    this.root.append(lig);
    return lig;
  }

  public Mechanism2d getMechanism2d() {
    return mech;
  }
}
