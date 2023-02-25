package frc.robot.subsystems.wrist;

import java.util.function.Function;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ARM;
import frc.robot.commands.arm.SetArmPose.ArmPose;

public enum WristState {
  //These values are ALL WRONG
  //arm 0 deg is positive x axis

  FEEDER_STATION(Units.radiansToDegrees(ARM.MIN_ANGLE), Units.radiansToDegrees(-3.16), -90),
  TRANSITION(Units.radiansToDegrees(-3.16), Units.radiansToDegrees(-2.47), 0),
  INSIDE_ROBOT(-140, -70, 90, WristStateType.OVERRIDE),
  RIGHT(-70, Units.radiansToDegrees(ARM.MAX_ANGLE), 90),

  GROUND_PICK_UP(-70, ArmPose.LOW_NODE.val+10, -90, WristStateType.ALT),
  PLACE_ON_NODE(ArmPose.LOW_NODE.val+10, Units.radiansToDegrees(ARM.MAX_ANGLE), 0, WristStateType.ALT);

  public enum WristStateType{
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

  private WristState(double min, double max, double goal, WristStateType type) {
    this(min, max, (Double pos) -> {
      return Units.degreesToRadians(goal);
    }, type);
  }

  private WristState(double min, double max, double goal) {
    this(min, max, (Double pos) -> {
      return Units.degreesToRadians(goal);
    }, WristStateType.DEFAULT);
  }

  private WristState(double min, double max, Function<Double, Double> getWristGoalLambda, WristStateType type) {
    this.min = Units.degreesToRadians(min);
    this.max = Units.degreesToRadians(max);
    this.getWristGoalLambda = getWristGoalLambda;
    this.type=type;
  }
  private WristState(double min, double max, Function<Double, Double> getWristGoalLambda) {
    this(min, max, getWristGoalLambda, WristStateType.DEFAULT);
  }
}
