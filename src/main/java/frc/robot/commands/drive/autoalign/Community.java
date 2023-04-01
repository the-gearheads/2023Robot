// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.autoalign;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public enum Community {
    BLUE_GRID(
        new Grid(
            new GridCol(
                new Translation2d(1.77,5.026),
                new Translation2d(2.163,5.024),
                new Translation2d(2.0,5.162)
            ),
            new GridCol(
                new Translation2d(1.833, 4.466),
                new Translation2d(2.129, 4.496),
                new Translation2d(2.03, 4.4)
            ),
            new GridCol(
                new Translation2d(1.763, 3.943),
                new Translation2d(2.188, 3.916),
                new Translation2d(1.99, 3.8)
            )
        ), 
        new Grid(
            new GridCol(
                new Translation2d(1.773,3.30),
                new Translation2d(2.173,3.34),
                new Translation2d(2.0,3.316)
            ),
            new GridCol(
                new Translation2d(1.833, 2.728),
                new Translation2d(2.129, 2.728),
                new Translation2d(2.03, 2.728)
            ),
            new GridCol(
                new Translation2d(1.77, 2.17),
                new Translation2d(2.17, 2.21),
                new Translation2d(1.99, 2.192)
            )
        ),
        new Grid(
            new GridCol(
                new Translation2d(1.776,1.69),
                new Translation2d(2.155,1.66),
                new Translation2d(2.0,1.68)
            ),
            new GridCol(
                new Translation2d(1.833, 1.1),
                new Translation2d(2.129, 1.1),
                new Translation2d(2.03, 1.1)
            ),
            new GridCol(
                new Translation2d(1.78, 0.46),
                new Translation2d(2.175, 0.45),
                new Translation2d(1.99, 0.554)
            )
        )
    ),
    RED_GRID(
        new Grid(//left
            new GridCol(//left
                new Translation2d(1.777,7.558),
                new Translation2d(2.13,7.515),
                new Translation2d(2.0,7.517)
            ),
            new GridCol(//center
                new Translation2d(1.833, 6.971),
                new Translation2d(2.129, 6.971),
                new Translation2d(2.03, 6.971)
            ),
            new GridCol(//right
                new Translation2d(1.795, 6.435),
                new Translation2d(2.145, 6.415),
                new Translation2d(1.99, 6.376)
            )
        ), 
        new Grid(//center
            new GridCol(
                new Translation2d(1.77,5.85),
                new Translation2d(2.135,5.855),
                new Translation2d(2.0,5.839)
            ),
            new GridCol(
                new Translation2d(1.833, 5.282),
                new Translation2d(2.129, 5.282),
                new Translation2d(2.03, 5.282)
            ),
            new GridCol(
                new Translation2d(1.8, 4.75),
                new Translation2d(2.196, 4.749),//we don't have this one
                new Translation2d(1.99, 4.749)
            )
        ),
        new Grid(//right
            new GridCol(
                new Translation2d(1.8,4.18),
                new Translation2d(2.158,4.12),
                new Translation2d(2.0,4.159)
            ),
            new GridCol(
                new Translation2d(1.833, 3.581),
                new Translation2d(2.129, 3.581),
                new Translation2d(2.03, 3.581)
            ),
            new GridCol(
                new Translation2d(1.75, 2.98),//3.05
                new Translation2d(2.09, 2.99),
                new Translation2d(1.99, 3.064)
            )
        )
    );

    public Grid leftGrid;
    public Grid centerGrid;
    public Grid rightGrid;

    private Community(Grid leftGrid, Grid centerGrid, Grid rightGrid){
        this.leftGrid = leftGrid;
        this.centerGrid = centerGrid;
        this.rightGrid = rightGrid;
    }
}
