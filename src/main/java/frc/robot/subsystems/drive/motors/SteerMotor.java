package frc.robot.subsystems.drive.motors;

public interface SteerMotor {
  /** Angle to set, in degrees */
  public void setAngle(double angle);
  public void setBrakeMode(boolean isBraking);

  public double getVelocity();
  public double getAngle();

  public void setPIDConstants(double kF, double kP, double kI, double kD);

  public default void updateFF(double FF) {};
  public default void updateP(double P) {};
  public default void updateI(double I) {};
  public default void updateD(double D) {};

  public default double getFF() {return 0.0;};
  public default double getP() {return 0.0;};
  public default double getI() {return 0.0;};
  public default double getD() {return 0.0;};
  
  default public void zeroEncoders() {}
}
