// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autoalign;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public enum Grid {
    LEFT_GRID(
        new GridCol(
            new Translation2d(1.837,5.05),
            new Translation2d(2.237,5.05),
            new Translation2d(2.0,5.162)
        ),
        new GridCol(
            new Translation2d(1.833, 4.466),
            new Translation2d(2.129, 4.496),
            new Translation2d(2.03, 4.4)
        ),
        new GridCol(
            new Translation2d(1.803, 3.87),
            new Translation2d(2.196, 3.82),
            new Translation2d(1.99, 3.8)
        )
    ), 
    CENTER_GRID(
        new GridCol(
            new Translation2d(1.837,5.039),
            new Translation2d(2.237,5.05),
            new Translation2d(2.0,5.162)
        ),
        new GridCol(
            new Translation2d(1.833, 4.466),
            new Translation2d(2.129, 4.496),
            new Translation2d(2.03, 4.4)
        ),
        new GridCol(
            new Translation2d(1.803, 3.82),
            new Translation2d(2.196, 3.82),
            new Translation2d(1.99, 3.8)
        )
    ),
    RIGHT_GRID(
        new GridCol(
            new Translation2d(1.837,5.039),
            new Translation2d(2.237,5.05),
            new Translation2d(2.0,5.162)
        ),
        new GridCol(
            new Translation2d(1.833, 4.466),
            new Translation2d(2.129, 4.496),
            new Translation2d(2.03, 4.4)
        ),
        new GridCol(
            new Translation2d(1.803, 3.82),
            new Translation2d(2.196, 3.82),
            new Translation2d(1.99, 3.8)
        )
    );

    
    public GridCol centerCol;
    public GridCol rightCol;
    public GridCol leftCol;

    private Grid(GridCol leftCol, GridCol centerCol, GridCol rightCol){
        this.leftCol=leftCol;
        this.centerCol=centerCol;
        this.rightCol=rightCol;
    }

    public double getY(){
        return this.centerCol.high.getY();
    }
}
