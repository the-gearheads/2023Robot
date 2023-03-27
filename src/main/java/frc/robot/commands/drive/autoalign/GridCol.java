// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autoalign;

import edu.wpi.first.math.geometry.Translation2d;

public class GridCol{
    public Translation2d low;
    public Translation2d mid;
    public Translation2d high;

    public GridCol(Translation2d high, Translation2d mid, Translation2d low){
        this.high=high;
        this.mid=mid;
        this.low=low;
    }
}
