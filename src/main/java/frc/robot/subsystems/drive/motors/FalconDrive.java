package frc.robot.subsystems.drive.motors;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DRIVE;

public class FalconDrive implements DriveMotor {

  private TalonFX motor;
  private DRIVE.DriveMotor drv = DRIVE.DRIVE_MOTOR;
  private String path;

  private static double ROTATIONS_TO_METERS = DRIVE.DRIVE_POS_FACTOR;
  private static double METERS_TO_ROTATIONS = 1d / DRIVE.DRIVE_POS_FACTOR;
  /* Yes this is intentional, Phoenix6 uses rot/sec rather than rot/min so we don't use the vel factor as that uses RPM.
   * See https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html */
  private static double RPS_TO_MPS = DRIVE.DRIVE_POS_FACTOR;
  private static double MPS_TO_RPS = 1d / DRIVE.DRIVE_POS_FACTOR;

  public FalconDrive(int id, String path) {
    this.motor = new TalonFX(id);
    this.path = path;
    configure();
  }


  public void configure() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = drv.DRIVE_PIDF[0];
    cfg.Slot0.kI = drv.DRIVE_PIDF[1];
    cfg.Slot0.kD = drv.DRIVE_PIDF[2];
    cfg.Slot0.kV = drv.DRIVE_PIDF[3];

    cfg.MotorOutput.DutyCycleNeutralDeadband = 0.005;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    cfg.CurrentLimits.StatorCurrentLimit = drv.CURRENT_LIMIT;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(cfg);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  private VelocityDutyCycle velDutyCycle = new VelocityDutyCycle(0, false, 0, 0, false);
  public void setSpeed(double speed) {
    motor.setControl(velDutyCycle.withVelocity(speed * MPS_TO_RPS));
  }

  public void zeroEncoders() {
  }

  public double getPosition() {
    return motor.getRotorPosition().getValue().doubleValue() * ROTATIONS_TO_METERS;
  }

  public double getVelocity() {
    return motor.getRotorVelocity().getValue().doubleValue() * RPS_TO_MPS;
  }

  public double getVelocitySetpoint() {
    return 0;
  }

  public double getTemperature() {
    return motor.getDeviceTemp().getValue();
  }

  public double getAppliedVolts() {
    return motor.getDutyCycle().getValue() * motor.getSupplyVoltage().getValue();
  }
}
