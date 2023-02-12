package frc.robot.subsystems;

import javax.swing.TransferHandler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class test {
    public static void main(String[] args){
        Pose2d initial = new Pose2d();
        Transform2d transform = new Transform2d(new Translation2d(5,0), Rotation2d.fromDegrees(180));
        Pose2d finalPos = initial.transformBy(transform);
        System.out.println(finalPos.getX() + " " + finalPos.getY());
    }
    
}
