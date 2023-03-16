// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import frc.robot.util.MoreMath;
import frc.robot.util.vision.SimCamParams;
import frc.robot.util.vision.SimVisionSystem;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

/** Add your docs here. */
public class VisionSim extends Vision{
    ArrayList<SimVisionSystem> simCams;
    private Field2d field;
    private Field2d staticField;
    private Pose2d actualStaticPose = new Pose2d(1.5,1.5, Rotation2d.fromDegrees(90));
    public VisionSim(Swerve swerve){
        super(swerve);
        initSimCams();

        this.field = new Field2d();
        SmartDashboard.putData("vision sim field", field);

        this.staticField = new Field2d();
        SmartDashboard.putData("vision sim delta field", staticField);
        staticField.setRobotPose(this.actualStaticPose);
    }

    @Override
    public void simulationPeriodic(){
        for(var simCam: simCams){
            simCam.clearVisionTargets();
            simCam.addVisionTargets(getAtfl());
            simCam.processFrame(swerve.getWheelPose());
        }
        
        this.field.setRobotPose(this.swerve.getWheelPose());

        var estimate = getEstimate();
        if(isEstimatePresent(estimate)){
            var pose = estimate.get().best.toPose2d();
            this.field.getObject("vision est").setPose(pose);
            Logger.getInstance().recordOutput("Vision/logging vision est", true);

            var staticTrans = pose.getTranslation().minus(this.swerve.getWheelPose().getTranslation()).plus(this.actualStaticPose.getTranslation());
            var staticPose =new Pose2d(staticTrans, this.actualStaticPose.getRotation());
            this.staticField.getObject("vision est").setPose(staticPose);
        }else{
            Logger.getInstance().recordOutput("Vision/logging vision est", false);
        }
    }

    public void initSimCams(){
        simCams = new ArrayList<SimVisionSystem>();
        for(SimCamParams simCamParams: Constants.VISION_SIM.SIM_CAMS_PARAMS){
            var simCam = 
            new SimVisionSystem(
                    simCamParams.name,
                    simCamParams.camDiagFOV,
                    simCamParams.robot2Cam,
                    simCamParams.maxLEDRange,
                    simCamParams.camResWidth,
                    simCamParams.camResHeight,
                    simCamParams.minTargetArea,0.1);
            simCams.add(simCam);

            double hypotPixels = Math.hypot(simCamParams.camResWidth, simCamParams.camResHeight);
            var camHorizFOVDegrees = simCamParams.camDiagFOV * simCamParams.camResWidth / hypotPixels;
            var camVertFOVDegrees = simCamParams.camDiagFOV * simCamParams.camResHeight / hypotPixels;
            var camMatrix = MoreMath.calcCamMatrix(simCamParams.camResWidth,
             simCamParams.camResHeight, 
             camHorizFOVDegrees, camVertFOVDegrees);
            simCam.getSimCam().setCameraIntrinsicsMat(camMatrix);
            simCam.getSimCam().setCameraDistortionMat(VecBuilder.fill(0,0,0,0,0));
        }
    }

    public Optional<CustomEstimate> getEstimate(){
        Optional<CustomEstimate> latestEstimate = Optional.empty();
        for(var estimate : estimates){
            if(!isEstimatePresent(latestEstimate) ||
                (isEstimatePresent(estimate) && estimate.get().timestampSeconds>latestEstimate.get().timestampSeconds)){
                latestEstimate=estimate;
            }
        }
        return latestEstimate;
    }
}
