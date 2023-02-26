package frc.robot.subsystems.wrist;

import java.util.function.Function;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ARM;
import frc.robot.commands.arm.SetArmPose.ArmPose;

public enum WristState {
  //These values are ALL WRONG
  //arm 0 deg is positive x axis
  // format:off
  FEEDER_STATION(ARM.MIN_ANGLE, -170, -90), 
  TRANSITION(-170, -140,0),
  INSIDE_ROBOT(-140, -70, 90, WristStateType.OVERRIDE), 
  RIGHT(-70, ARM.MAX_ANGLE, 90),

  GROUND_PICK_UP(-66, -50, -90, WristStateType.ALT), 
  PLACE_ON_NODE(-50, ARM.MAX_ANGLE, 0, WristStateType.ALT);
  // format:on

  public enum WristStateType {
    OVERRIDE, ALT, DEFAULT;
  }

  private Function<Double, Double> getWristGoalLambda;
  private double min;
  private double max;
  public WristStateType type;

  public boolean inRange(double currentWrappedPos) {
    return currentWrappedPos >= min && currentWrappedPos <= max;
  }

  public double getWristGoal(double currentWrappedPos) {
    return this.getWristGoalLambda.apply(currentWrappedPos);
  }

  
  private WristState(double min, double max, double goal) {
    this(min,max,goal,WristStateType.DEFAULT);
  }
  private WristState(double min, double max, double goal, WristStateType type) {
    this(min, max, (Double pos) -> {
      return goal;
    }, type);
  }

  private WristState(double min, double max, Function<Double, Double> getWristGoalLambda) {
    this(min, max, getWristGoalLambda, WristStateType.DEFAULT);
  }
  private WristState(double min, double max, Function<Double, Double> getWristGoalLambda, WristStateType type) {
    this.min = min;
    this.max = max;
    this.getWristGoalLambda = getWristGoalLambda;
    this.type = type;
  }
}
