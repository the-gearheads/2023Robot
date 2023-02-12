package frc.robot.subsystems;

import java.util.function.Function;
import edu.wpi.first.math.util.Units;

public enum WristState{
    //These values are ALL WRONG
    //arm 0 deg is positive x axis

    LEFT(-225, -120, 180),
    UP(-120,-70,90),
    LINEAR(-70,-50,(Double armPos)->{
        return armPos;
    }),
    RIGHT(-50,45, 0);

    private Function<Double, Double> getWristGoalLambda;
    private double min;
    private double max;

    public boolean inRange(double currentWrappedPos){
        return currentWrappedPos>=min && currentWrappedPos<=max;
    }
    public double getWristGoal(double currentWrappedPos){
        return (this.getWristGoalLambda.apply(currentWrappedPos));
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
