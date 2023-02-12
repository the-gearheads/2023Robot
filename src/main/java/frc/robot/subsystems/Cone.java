// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class Cone {
    ArrayList<Corner> corners = new ArrayList<Corner>();
    private Pose2d pose;
    private MechRootWrapper parent;
    private MechRootWrapper root;
    public Cone(MechRootWrapper parent, Pose2d pose){
        this.parent=parent;
        this.pose=pose;
        createCone();
    }

    public ArrayList<Pose2d> calculateCornerPoses(ArrayList<Double> sideLengths){
        ArrayList<Pose2d> cornerPoses = new ArrayList<>();
        ArrayList<Rotation2d> interiorAngles = calculateInteriorAngles(sideLengths);

        Pose2d bottomLeftCorner = new Pose2d(0,0, new Rotation2d());
        Pose2d bottomRightCorner = new Pose2d(new Translation2d(sideLengths.get(0), 0),
                                        new Rotation2d(Math.PI).minus(interiorAngles.get(1)));
        Pose2d topCorner = new Pose2d(new Translation2d(sideLengths.get(1), interiorAngles.get(0)),
                                        new Rotation2d(Math.PI).plus(interiorAngles.get(0)));

        cornerPoses.add(bottomLeftCorner);
        cornerPoses.add(bottomRightCorner);
        cornerPoses.add(topCorner);

        double xAverage = cornerPoses.stream().mapToDouble(cornerPose -> cornerPose.getX()).average().getAsDouble();
        double yAverage = cornerPoses.stream().mapToDouble(cornerPose -> cornerPose.getY()).average().getAsDouble();

        Pose2d centroid = new Pose2d(xAverage, yAverage, new Rotation2d());

        for(int i = 0; i < 3; i++){
            Pose2d cornerPose = cornerPoses.get(i);
            cornerPoses.set(i,cornerPose.relativeTo(centroid));
        }
        return cornerPoses;
    }

    public ArrayList<Rotation2d> calculateInteriorAngles(ArrayList<Double> sideLengths){
        ArrayList<Rotation2d> interiorAngles = new ArrayList<>();
        for(int i = 0; i < 3; i++){
            double currentSide = sideLengths.get((i+1)%3);
            double otherSide = sideLengths.get((i+2)%3);
            double otherSide2 = sideLengths.get((i+3)%3);
            double angle = Math.acos((Math.pow(currentSide,2)
            -Math.pow(otherSide,2)
            -Math.pow(otherSide2,2))
            /(-2*otherSide*otherSide2));
            interiorAngles.add(new Rotation2d(angle));
        }
        return interiorAngles;
    }

    public void createCone(){
        Mechanism2d mech=this.parent.getMechanism2d();
        this.root = new MechRootWrapper(mech, "Cone", 0, 0);

        ArrayList<Double> sideLengths = new ArrayList<Double>(){{
            add(5.0);
            add(5.0);
            add(5.0);
        }};
        ArrayList<Pose2d> cornerPoses = calculateCornerPoses(sideLengths);
        
        for(int i =0; i < 3; i++){
            corners.add(new Corner(i,root, cornerPoses.get(i), sideLengths.get(i)));
        }
    }

    public void update(Rotation2d rot){
        Pose2d parentPos = new Pose2d(parent.getPosition().getTranslation(), rot);
        Transform2d deltaTransform = new Transform2d(this.pose.getTranslation(), this.pose.getRotation());
        Pose2d newConePose = parentPos.transformBy(deltaTransform);
        this.root.setPosition(newConePose.getX(), newConePose.getY());

        for(Corner corner:corners){
            corner.update(rot);
        }
    }

    public class Corner{
        private MechRootWrapper root;
        private MechanismLigament2d side;
        private Pose2d pose;
        private MechRootWrapper parent;

        public Corner(int id, MechRootWrapper parent, Pose2d pose, double sideLength){
            this.parent=parent;
            this.pose=pose;
            root = new MechRootWrapper(parent.getMechanism2d(), "Side " + id, pose.getX(), pose.getY());
            side=root.append(new MechanismLigament2d("Side " + id, 5, 0,20.0,new Color8Bit(Color.kYellow)));
        }

        public void update(Rotation2d rot){
            Pose2d parentPos = new Pose2d(this.parent.getPosition().getTranslation(), rot);
            Transform2d deltaTransform = new Transform2d(this.pose.getTranslation(), new Rotation2d());
            Pose2d newPose = parentPos.transformBy(deltaTransform);
            root.setPosition(newPose.getX(), newPose.getY());
            side.setAngle(rot.plus(pose.getRotation()));
        }
    }
}
