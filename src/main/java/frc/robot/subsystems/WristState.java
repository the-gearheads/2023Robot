package frc.robot.subsystems;

import java.util.function.Function;
import edu.wpi.first.math.util.Units;

public enum WristState{
    //These values are ALL WRONG
    //arm 0 deg is positive x axis
    UP(150,200, 0),
    LINEAR(110,150,(Double armPos)->{
        return armPos;
    }),
    LEFT(0, 110, 90),
    RIGHT(200,360,270);

    private Function<Double, Double> getWristGoalLambda;
    private double min;
    private double max;

    public boolean inRange(double currentWrappedPos){
        return currentWrappedPos>=min && currentWrappedPos<=max;
    }
    public double getWristGoal(double currentWrappedPos){
        return this.getWristGoalLambda.apply(currentWrappedPos);
    }
    private WristState(double min, double max, double goal){
        this(min, max, (Double pos)->{
        return Units.degreesToRadians(goal);
        });
    }
    private WristState(double min, double max, Function<Double, Double> getWristGoalLambda){
        this.min=Units.degreesToRadians(min);
        this.max=Units.degreesToRadians(max);
        this.getWristGoalLambda=getWristGoalLambda;
    }
  }
