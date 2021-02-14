package org.frcteam2910.c2020.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.IOException;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory barrelRacingAutoNavPartOne;
    private Trajectory barrelRacingAutoNavPartTwo;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        barrelRacingAutoNavPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(42.75,90),Rotation2.ZERO)
                .lineTo(new Vector2(150,90))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        barrelRacingAutoNavPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(150,90),Rotation2.ZERO)
                    .arcTo(new Vector2(175.98,45),new Vector2(150,60))
                    .arcTo(new Vector2(124.02,45),new Vector2(150,60))
                    .arcTo(new Vector2(150,90),new Vector2(150,60))
                    .lineTo(new Vector2(240,90))
                    .arcTo(new Vector2(251.48,147.72),new Vector2(240,120))
                    .arcTo(new Vector2(218.79,98.79),new Vector2(240,120))
                    .lineTo(new Vector2(278.79,38.79))
                    .arcTo(new Vector2(327.72,48.52),new Vector2(300,60))
                    .arcTo(new Vector2(300,90),new Vector2(300,60))
                    .lineTo(new Vector2(-60,90))
                    .build(),
            trajectoryConstraints, SAMPLE_DISTANCE
        );

    }

    public Trajectory getBarrelRacingAutoNavPartOne(){
        return barrelRacingAutoNavPartOne;
    }

    public Trajectory getBarrelRacingAutoNavPartTwo(){
        return barrelRacingAutoNavPartTwo;
    }
}
