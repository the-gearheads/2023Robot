// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.vision.CustomEstimate;
import frc.robot.subsystems.vision.Vision;

public class FuseVisionEstimate extends CommandBase {
  private Vision vision;

  public enum ConfidenceStrat {
    MECH_ADV, IRON_PANTHERS, CASSEROLE, TEST, NONE;
  }

  public ConfidenceStrat confidenceStrat;
  public SendableChooser<ConfidenceStrat> trustChooser;

  /** Creates a new UpdateSwervePoseEstimator. */
  public FuseVisionEstimate(Vision vision) {
    this(vision, ConfidenceStrat.NONE);
  }

  public FuseVisionEstimate(Vision vision, ConfidenceStrat confidenceStrat) {
    this.vision = vision;
    this.confidenceStrat = confidenceStrat;
    initChooser();
    addRequirements(vision);
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
          fuseEstimate(estimate);
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
}
