package frc.robot.subsystems.wrist;

import frc.robot.Constants.ARM;

public enum WristState {

  //arm 0 deg is positive x axis
  // @formatter:off

  /* DEFAULT */
  FEEDER_STATION(ARM.MIN_ANGLE, -170, -110.7), 
  TRANSITION(-170, -140, 0), 
  INSIDE_ROBOT(-140, -81, 90), 
  PLACE(-81,ARM.MAX_ANGLE, 90),

  /* ALT */
  GROUND_PICK_UP(-66, -50, -90, WristControlType.ALT),
  PLACE_ON_NODE(-50, ARM.MAX_ANGLE, 0, WristControlType.ALT),

  /* MANUAL */
  UP(90, WristControlType.MANUAL), 
  DOWN(-90, WristControlType.MANUAL), 
  RIGHT(0, WristControlType.MANUAL), 
  VARIABLE(0, WristControlType.MANUAL);//goal here is unnecessary, since we will use the setWristGoal method 
  //LEFT is illegal

  // @formatter:on

  public enum WristControlType {
    ALT, DEFAULT, MANUAL;
  }

  private final double min;
  private final double max;
  public final WristControlType type;
  private double goal;

  private WristState(double goal, WristControlType type) {
    this(ARM.MIN_ANGLE, ARM.MAX_ANGLE, goal, type);
  }

  private WristState(double min, double max, double goal) {
    this(min, max, goal, WristControlType.DEFAULT);
  }

  private WristState(double min, double max, double goal, WristControlType type) {
    this.min = min;
    this.max = max;
    this.goal = goal;
    this.type = type;
  }

  public boolean inRange(double currentWrappedPos) {
    return currentWrappedPos >= min && currentWrappedPos <= max;
  }

  private void setGoal(double goal) {
    this.goal = goal;
  }

  public double getGoal() {
    return goal;
  }

  //this is cursed
  public static WristState getStateWithGoal(double goal) {
    VARIABLE.setGoal(goal);
    return VARIABLE;
  }
}
