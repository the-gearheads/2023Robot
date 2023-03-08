package frc.robot.subsystems.wrist;

import java.util.function.Function;
import frc.robot.Constants.ARM;

public enum WristState {

  //arm 0 deg is positive x axis
  // format:off

  /* DEFAULT */
  FEEDER_STATION(ARM.MIN_ANGLE, -170, -110.7), TRANSITION(-170, -140, 0), INSIDE_ROBOT(-140, -70, 90), PLACE(-70,
      ARM.MAX_ANGLE, 90),

  /* ALT */
  GROUND_PICK_UP(-66, -50, -90, WristControlType.ALT), PLACE_ON_NODE(-50, ARM.MAX_ANGLE, 0, WristControlType.ALT),

  /* MANUAL */
  UP(90, WristControlType.MANUAL), DOWN(-90, WristControlType.MANUAL), RIGHT(0, WristControlType.MANUAL), VARIABLE(0,
      WristControlType.MANUAL);//goal here is unnecessary, since we will use the setWristGoal method 
  //LEFT is illegal

  // format:on
  public enum WristControlType {
    ALT, DEFAULT, MANUAL;
  }

  private Function<Double, Double> getWristGoalLambda;
  private double min;
  private double max;
  public WristControlType type;

  public boolean inRange(double currentWrappedPos) {
    return currentWrappedPos >= min && currentWrappedPos <= max;
  }

  public double getWristGoal(double currentWrappedPos) {
    return this.getWristGoalLambda.apply(currentWrappedPos);
  }

  public void setWristGoal(double goal) {
    this.getWristGoalLambda = (armPose) -> goal;
  }

  private WristState(double goal, WristControlType type) {
    this(ARM.MIN_ANGLE, ARM.MAX_ANGLE, goal, type);
  }

  private WristState(double min, double max, double goal) {
    this(min, max, goal, WristControlType.DEFAULT);
  }

  private WristState(double min, double max, double goal, WristControlType type) {
    this(min, max, (Double pos) -> {
      return goal;
    }, type);
  }

  private WristState(double min, double max, Function<Double, Double> getWristGoalLambda) {
    this(min, max, getWristGoalLambda, WristControlType.DEFAULT);
  }

  private WristState(double min, double max, Function<Double, Double> getWristGoalLambda, WristControlType type) {
    this.min = min;
    this.max = max;
    this.getWristGoalLambda = getWristGoalLambda;
    this.type = type;
  }
}
