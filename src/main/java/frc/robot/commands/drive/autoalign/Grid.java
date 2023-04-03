// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autoalign;

/** Add your docs here. */
public class Grid {
    public GridCol centerCol;
    public GridCol rightCol;
    public GridCol leftCol;

    public Grid(GridCol leftCol, GridCol centerCol, GridCol rightCol){
        this.leftCol=leftCol;
        this.centerCol=centerCol;
        this.rightCol=rightCol;
    }

    public double getY(){
        return this.centerCol.high.getY();
    }
}
