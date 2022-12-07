package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class test {
    public static void main(String... args){
        SwerveModuleState state = new SwerveModuleState(1, Rotation2d.fromDegrees(-179));
        SwerveModuleState newState = optimize(state, Rotation2d.fromDegrees(179));
        System.out.println(newState.angle.getDegrees());
    }
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        System.out.println(delta.getDegrees());
        if (Math.abs(delta.getDegrees()) > 90.0) {
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }
}
