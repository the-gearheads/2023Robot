package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyroscope extends AHRS{
    private boolean isInverted;
    public Gyroscope(SPI.Port port){
        this(port, false);
    }

    public Gyroscope(SPI.Port port, boolean isInverted){
        super(port);
        this.isInverted = isInverted;
    }
    public void setInverted(boolean isInverted){
        this.isInverted = false;
    }
    public Rotation2d getRotation2d(){
        double direction = isInverted ? 1 : -1;
        return new Rotation2d(direction * super.getRotation2d().getRadians());
    }
    public void setRotation2d(Rotation2d newRotation2d){
        double direction = isInverted ? -1 : 1;
        double newAngle = newRotation2d.getDegrees() * direction;
        super.zeroYaw();
        super.setAngleAdjustment(newAngle);
    }
    public double getContinuousAngle(){
        double direction = isInverted ? -1 : 1;
        return direction * super.getAngle();
    }
}