package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableSparkMaxPID implements Sendable {
  SparkMaxPIDController pid;
  double lastP, lastI, lastD, lastFF;

  public SendableSparkMaxPID(SparkMaxPIDController pid) {
    this.pid = pid;
  }

  @Override
  public void initSendable(SendableBuilder b) {
    b.addDoubleProperty("P", this::getP, this::setP);
    b.addDoubleProperty("I", this::getI, this::setI);
    b.addDoubleProperty("D", this::getD, this::setD);
    b.addDoubleProperty("FF", this::getFF, this::setFF);
  }

  public REVLibError setP(double newP) {
    lastP = newP;
    return pid.setP(newP);
  }

  public REVLibError setI(double newI) {
    lastI = newI;
    return pid.setI(newI);
  }

  public REVLibError setD(double newD) {
    return pid.setD(newD);
  }

  public REVLibError setFF(double newFF) {
    return pid.setFF(newFF);
  }

  public double getP() {
    return lastP;
  }

  public double getI() {
    return lastI;
  }

  public double getD() {
    return lastD;
  }

  public double getFF() {
    return lastFF;
  }
}
