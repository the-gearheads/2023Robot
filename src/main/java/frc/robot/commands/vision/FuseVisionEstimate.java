// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.MoreMath;
import frc.robot.util.vision.CustomEstimate;
import frc.robot.Constants.AUTO_ALIGN.COMMUNITY;
import frc.robot.Constants.AUTO_ALIGN.FEEDER;
import frc.robot.subsystems.vision.Vision;

public class FuseVisionEstimate extends CommandBase {
  private Vision vision;

  public enum ConfidenceStrat {
    MECH_ADV, IRON_PANTHERS, CASSEROLE, TEST, NONE, ONLY_COMMUNITY_AND_FEEDER, ONLY_COMMUNITY;
  }

  public ConfidenceStrat confidenceStrat;
  public SendableChooser<ConfidenceStrat> trustChooser;

  /** Creates a new UpdateSwervePoseEstimator. */
  public FuseVisionEstimate(Vision vision) {
    this(vision, ConfidenceStrat.ONLY_COMMUNITY_AND_FEEDER);
  }

  public FuseVisionEstimate(Vision vision, ConfidenceStrat confidenceStrat) {
    this.vision = vision;
    this.confidenceStrat = confidenceStrat;
    addRequirements(vision);
  }

  @Override
  public void initialize(){
    initChooser();
  }

  private void initChooser() {
    trustChooser = new SendableChooser<ConfidenceStrat>();
    for (var visionTrustType : ConfidenceStrat.values()) {
      var key = visionTrustType.name();
      var val = visionTrustType;
      trustChooser.addOption(key, val);
    }

    trustChooser.setDefaultOption(confidenceStrat.name(), confidenceStrat);
    SmartDashboard.putData("vision/trust chooser", trustChooser);
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    confidenceStrat = trustChooser.getSelected();
    for (var optEstimate : vision.estimates) {
      if (optEstimate.isEmpty())
        continue;
      var estimate = optEstimate.get();
      Matrix<N3, N1> confidence;
      switch (confidenceStrat) {
        case IRON_PANTHERS:
          confidence = VisionHelper.ironPanthersStdDevs(estimate);
          estimate.setConfidence(confidence);
          fuseEstimate(estimate);
          break;
        case CASSEROLE:
          confidence = VisionHelper.ROBOT_CASSEROLE_STDEV;
          estimate.setConfidence(confidence);
          if (VisionHelper.casseroleShouldFuse(estimate, vision)) {
            fuseEstimate(estimate);
          }
          break;
        case MECH_ADV:
          estimate.setConfidence(VisionHelper.mechAdvStdDevs(estimate));
          fuseEstimate(estimate);
          break;
        case TEST:
          var stDevs = VisionHelper.ironPanthersStdDevs(estimate);
          stDevs.set(2,0,Double.POSITIVE_INFINITY);
          estimate.setConfidence(stDevs);
          fuseEstimate(estimate);
          break;
        case ONLY_COMMUNITY:
        var stDev1 = VisionHelper.ironPanthersStdDevs(estimate);
        stDev1.set(2,0,Double.POSITIVE_INFINITY);
        estimate.setConfidence(stDev1);
        if(inCommunity(estimate.best.toPose2d())){
          SmartDashboard.putBoolean("fusing", true);
          fuseEstimate(estimate);
        }else{
          SmartDashboard.putBoolean("fusing", false);
        }
        break;
        case ONLY_COMMUNITY_AND_FEEDER:
          var stDev = VisionHelper.ironPanthersStdDevs(estimate);
          stDev.set(2,0,Double.POSITIVE_INFINITY);
          estimate.setConfidence(stDev);
          if(inCommunity(estimate.best.toPose2d()) || inFeederArea(estimate.best.toPose2d())){
            SmartDashboard.putBoolean("fusing", true);
            fuseEstimate(estimate);
          }else{
            SmartDashboard.putBoolean("fusing", false);
          }
          break;
        default:
          break;
      }
    }
  }

  public void fuseEstimate(CustomEstimate estimate) {
    var poseEstimate = estimate.best.toPose2d();
    var timestamp = estimate.timestampSeconds;
    var confidence = estimate.confidence;
    this.vision.swerve.fuseVisionEstimate(poseEstimate, timestamp, confidence);
  }

  private static boolean inCommunity(Pose2d pose) {
    var firstCorner = MoreMath.transformByAlliance(COMMUNITY.DIAG_CORNERS.get(0));
    var secondCorner = MoreMath.transformByAlliance(COMMUNITY.DIAG_CORNERS.get(1));

    return MoreMath.within(pose, firstCorner, secondCorner);
  }

  private static boolean inFeederArea(Pose2d pose) {
    var firstCorner = MoreMath.transformByAlliance(FEEDER.DIAG_CORNERS.get(0));
    var secondCorner = MoreMath.transformByAlliance(FEEDER.DIAG_CORNERS.get(1));

    return MoreMath.within(pose, firstCorner, secondCorner);
  }
}
