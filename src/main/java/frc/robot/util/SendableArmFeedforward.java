// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as a motor acting against the force of gravity on a beam suspended at an angle).
 */
public class SendableArmFeedforward implements Sendable {
  public double ks;
  public double kg;
  public double kv;
  public double ka;

  @Override
  public void initSendable(SendableBuilder b) {
    b.addDoubleProperty("kS", () -> {
      return ks;
    }, (double newKs) -> {
      ks = newKs;
    });
    b.addDoubleProperty("kG", () -> {
      return kg;
    }, (double newKg) -> {
      kg = newKg;
    });
    b.addDoubleProperty("kV", () -> {
      return kv;
    }, (double newKv) -> {
      kv = newKv;
    });
    b.addDoubleProperty("kA", () -> {
      return ka;
    }, (double newKa) -> {
      ka = newKa;
    });
  }

  /**
   * Creates a new ArmFeedforward with the specified gains. Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   * @param ka The acceleration gain.
   */
  public SendableArmFeedforward(double ks, double kg, double kv, double ka) {
    this.ks = ks;
    this.kg = kg;
    this.kv = kv;
    this.ka = ka;
  }

  /**
   * Creates a new ArmFeedforward with the specified gains. Acceleration gain is defaulted to zero. Units of the gain values will dictate units of the computed feedforward.
   *
   * @param ks The static gain.
   * @param kg The gravity gain.
   * @param kv The velocity gain.
   */
  public SendableArmFeedforward(double ks, double kg, double kv) {
    this(ks, kg, kv, 0);
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param positionDeg The position (angle) setpoint. This angle should be measured from the horizontal (i.e. if the provided angle is 0, the arm should be parallel with
   *          the floor). If your encoder does not follow this convention, an offset should be added.
   * @param velocityDegPerSec The velocity setpoint.
   * @param accelDegPerSecSquared The acceleration setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double positionDeg, double velocityDegPerSec, double accelDegPerSecSquared) {
    double positionRad = Units.degreesToRadians(positionDeg);
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double accelRadPerSecSquared = Units.degreesToRadians(accelDegPerSecSquared);

    return ks * Math.signum(velocityRadPerSec) + kg * Math.cos(positionRad) + kv * velocityRadPerSec
        + ka * accelRadPerSecSquared;
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to be zero).
   *
   * @param positionDeg The position (angle) setpoint. This angle should be measured from the horizontal (i.e. if the provided angle is 0, the arm should be parallel with
   *          the floor). If your encoder does not follow this convention, an offset should be added.
   * @param velocityDegPerSec The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double positionDeg, double velocityDegPerSec) {
    return calculate(positionDeg, velocityDegPerSec, 0);
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply, a position, and an acceleration. Useful for ensuring that velocity and acceleration
   * constraints for a trapezoidal profile are simultaneously achievable - enter the acceleration constraint, and this will give you a simultaneously-achievable velocity
   * constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param positionDeg The angle of the arm. This angle should be measured from the horizontal (i.e. if the provided angle is 0, the arm should be parallel with the
   *          floor). If your encoder does not follow this convention, an offset should be added.
   * @param accelDegPerSec The acceleration of the arm.
   * @return The maximum possible velocity at the given acceleration and angle.
   */
  public double maxAchievableVelocity(double maxVoltage, double positionDeg, double accelDegPerSec) {
    // Assume max velocity is positive
    double positionRad = Units.degreesToRadians(positionDeg);
    double accelRadPerSecSquared = Units.degreesToRadians(accelDegPerSec);
    return (maxVoltage - ks - Math.cos(positionRad) * kg - accelRadPerSecSquared * ka) / kv;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply, a position, and an acceleration. Useful for ensuring that velocity and acceleration
   * constraints for a trapezoidal profile are simultaneously achievable - enter the acceleration constraint, and this will give you a simultaneously-achievable velocity
   * constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param positionDeg The angle of the arm. This angle should be measured from the horizontal (i.e. if the provided angle is 0, the arm should be parallel with the
   *          floor). If your encoder does not follow this convention, an offset should be added.
   * @param accelDegPerSec The acceleration of the arm.
   * @return The minimum possible velocity at the given acceleration and angle.
   */
  public double minAchievableVelocity(double maxVoltage, double positionDeg, double accelDegPerSec) {
    double positionRad = Units.degreesToRadians(positionDeg);
    double accelRadPerSecSquared = Units.degreesToRadians(accelDegPerSec);
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + ks - Math.cos(positionRad) * kg - accelRadPerSecSquared * ka) / kv;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply, a position, and a velocity. Useful for ensuring that velocity and acceleration
   * constraints for a trapezoidal profile are simultaneously achievable - enter the velocity constraint, and this will give you a simultaneously-achievable acceleration
   * constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param positionDeg The angle of the arm. This angle should be measured from the horizontal (i.e. if the provided angle is 0, the arm should be parallel with the
   *          floor). If your encoder does not follow this convention, an offset should be added.
   * @param velocityDegPerSec The velocity of the arm.
   * @return The maximum possible acceleration at the given velocity.
   */
  public double maxAchievableAcceleration(double maxVoltage, double positionDeg, double velocityDegPerSec) {
    double positionRad = Units.degreesToRadians(positionDeg);
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    return (maxVoltage - ks * Math.signum(velocityRadPerSec) - Math.cos(positionRad) * kg - velocityRadPerSec * kv)
        / ka;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage supply, a position, and a velocity. Useful for ensuring that velocity and acceleration
   * constraints for a trapezoidal profile are simultaneously achievable - enter the velocity constraint, and this will give you a simultaneously-achievable acceleration
   * constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param positionDeg The angle of the arm. This angle should be measured from the horizontal (i.e. if the provided angle is 0, the arm should be parallel with the
   *          floor). If your encoder does not follow this convention, an offset should be added.
   * @param velocityDegPerSec The velocity of the arm.
   * @return The minimum possible acceleration at the given velocity.
   */
  public double minAchievableAcceleration(double maxVoltage, double positionDeg, double velocityDegPerSec) {
    return maxAchievableAcceleration(-maxVoltage, positionDeg, velocityDegPerSec);
  }
}
