// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.HashMap;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.auton.AutonChooser;
import frc.robot.auton.AutonHelper;
import frc.robot.subsystems.leds.LedState;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.MoreMath;
import frc.robot.util.vision.CustomEstimate;

public class AutonSetupBuddy extends CommandBase {

  private static final double transThreshold = 0.05;
  private static final double rotThreshold = 0.05;
  private HashMap<String, String> initPaths = new HashMap<String, String>(){{
    put("N4 Place Then Dock", "InertN4-StartN4");
    put("NO BUMP Place Then Move", "InertN1-StartN1");
    put("BUMP Place Then Move", "InertN9-StartN9");
    put("2 cone", "InertN1-StartN1");
    put("BUMP 2 cone", "InertN9-StartN9");
    put("NO BUMP Safe two cone", "InertN1-StartN1");
    put("Middle Go over station and grab, then dock", "InertN4-StartN4");
  }};
  private Vision vision;
  private Leds leds;

  /** Creates a new AutonSetupBuddy. */
  public AutonSetupBuddy(Vision vision, Leds leds) {
    this.vision = vision;
    this.leds = leds;
    addRequirements(vision, leds);
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }

  private Optional<Pose2d> getTrackedPose(){
    var autonsMap = AutonChooser.autons;
    var chosenAutonCommand = AutonChooser.chooser.getSelected();
    var chosenAutonName = MoreMath.getKeyByValue(autonsMap, chosenAutonCommand);

    if(chosenAutonName == null || !initPaths.containsKey(chosenAutonName)){
      return Optional.empty();
    }else{
      var initPathName = initPaths.get(chosenAutonName);
      var initPose = AutonHelper.getPathByName(initPathName, Constants.AUTON.SLOW_CONSTRAINTS).getInitialPose();
      if(initPose==null){
        return Optional.empty();
      }else{
        return Optional.of(initPose);
      }
    }
  }

  public Optional<Pose2d> getEstimatedPose(){
    var estimates = vision.estimates;

    if (estimates.isEmpty()) return Optional.empty();

    double totalAmbiguity=0;
    for (Optional<CustomEstimate> optEstimate : estimates) {
      if(optEstimate.isEmpty())continue;
      var estimate = optEstimate.get();
      double ambiguity = estimate.ambiguity;

      // Pose ambiguity is 0, use that pose
      if (ambiguity == 0) {
          return Optional.of(estimate.best.toPose2d());
      }

      totalAmbiguity += 1.0 / ambiguity;
  }

    Translation3d transform = new Translation3d();
    Rotation3d rotation = new Rotation3d();

    for (Optional<CustomEstimate> optEstimate : estimates) {
      if(optEstimate.isEmpty())continue;
      var estimate = optEstimate.get();
      //total ambiguity can't be zero, because we checked that previously
      double weight = (1.0 / estimate.ambiguity) / totalAmbiguity;
      var estimatedPose = estimate.best;
      transform = transform.plus(estimatedPose.getTranslation().times(weight));
      rotation = rotation.plus(estimatedPose.getRotation().times(weight));
    }

    var pose3d = new Pose3d(transform, rotation);
    return Optional.of(pose3d.toPose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var trackedPoseOpt = getTrackedPose();
    var robotPoseOpt = getEstimatedPose();

    if(trackedPoseOpt.isEmpty()){
      leds.setState(LedState.RED);
      return;
    }else if(robotPoseOpt.isEmpty()){
      leds.setState(LedState.PINK);
      return;
    }

    var trackedPose = trackedPoseOpt.get();
    var robotPose = robotPoseOpt.get();

    var delta = trackedPose.minus(robotPose);
    var norm = delta.getTranslation().getNorm();
    var rotDist = delta.getRotation().getDegrees();
    
    if(norm > transThreshold){
      var angle = delta.getTranslation().getAngle().getDegrees();

      int cardinalDir = (int) (45 * Math.round(angle/45));
      switch(cardinalDir){
        case -180:
        leds.setState(LedState.YELLOW_SOUTH);
        break;
        case -135:
        leds.setState(LedState.YELLOW_SE);
        break;
        case -90:
        leds.setState(LedState.YELLOW_EAST);
        break;
        case -45:
        leds.setState(LedState.YELLOW_NE);
        break;
        case 0:
        leds.setState(LedState.YELLOW_NORTH);
        break;
        case 45:
        leds.setState(LedState.YELLOW_NW);
        break;
        case 90:
        leds.setState(LedState.YELLOW_WEST);
        break;
        case 135:
        leds.setState(LedState.YELLOW_SW);
        break;
        case 180:
        leds.setState(LedState.YELLOW_SOUTH);
        break;
        default:
        break;
      }
    }else if(Math.abs(rotDist) > rotThreshold){
      if(rotDist > 0){
        leds.setState(LedState.YELLOW_COUNTERCLOCKWISE);
      }else{
        leds.setState(LedState.YELLOW_CLOCKWISE);
      }
    }else{
      leds.setState(LedState.GREEN);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setState(LedState.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
