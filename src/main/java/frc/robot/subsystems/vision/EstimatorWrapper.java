// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class EstimatorWrapper {
    private Transform3d robot2Cam;
    private PhotonCamera cam;

    public EstimatorWrapper(PhotonCamera cam, Transform3d robot2Cam){
        this.cam = cam;
        this.robot2Cam = robot2Cam;
    }
}
