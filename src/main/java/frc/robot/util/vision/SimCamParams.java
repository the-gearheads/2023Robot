// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class SimCamParams {
    public double camDiagFOV;
    public int maxLEDRange;
    public int camResWidth;
    public int camResHeight;
    public int minTargetArea;
    public Transform3d robot2Cam;
    public String name;
    
    public SimCamParams(double camDiagFOV, int maxLEDRange, int camResolutionWidth, 
    int camResolutionHeight, int minTargetArea, Transform3d robot2Cam, String name) {
        this.camDiagFOV = camDiagFOV;
        this.maxLEDRange = maxLEDRange;
        this.camResWidth = camResolutionWidth;
        this.camResHeight = camResolutionHeight;
        this.minTargetArea = minTargetArea;
        this.robot2Cam = robot2Cam;
        this.name = name;
    }
}
