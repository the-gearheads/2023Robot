package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants.DRIVE;

/* Limit slew rate of polar representation of speed (0-1), to try and avoid tread delamination. 
 * Adapted from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java#L125-L159
 */
public class SwerveRateLimit {
  private SlewRateLimiter magLimiter = new SlewRateLimiter(DRIVE.MAG_SLEW_RATE);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DRIVE.ROT_SLEW_RATE);

  private double currentTranslationMag;
  private double currentTranslationDir;
  private double currentRotation;
  private double prevTime;
  
  public ChassisSpeeds rateLimit(ChassisSpeeds desiredSpeedPercentages) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(desiredSpeedPercentages.vyMetersPerSecond, desiredSpeedPercentages.vxMetersPerSecond);
      double inputTranslationMag = Math.sqrt(Math.pow(desiredSpeedPercentages.vxMetersPerSecond, 2) + Math.pow(desiredSpeedPercentages.vyMetersPerSecond, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DRIVE.DIRECTION_SLEW_RATE / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
      
      var xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      var ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(desiredSpeedPercentages.omegaRadiansPerSecond);
      return new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, currentRotation);
  }
}
