package org.frcteam2910.c2020.util;

import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.IOException;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory barrelRacingAutoNavPartOne;
    private Trajectory barrelRacingAutoNavPartTwo;

    private Trajectory bouncePathPartOne;
    private Trajectory bouncePathPartTwo;
    private Trajectory bouncePathPartThree;
    private Trajectory bouncePathPartFour;

    private Trajectory slalomPathPartOne;
    private Trajectory slalomPathPartTwo;

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

        bouncePathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(30,90),Rotation2.ZERO)
                .lineTo(new Vector2(60,90))
                .arcTo(new Vector2(90,120),new Vector2(60,120))
                .lineTo(new Vector2(90,125.38))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(90,125.38),Rotation2.ZERO)
                .lineTo(new Vector2(122.34,48.38))
                .arcTo(new Vector2(150,30),new Vector2(15,60))
                .arcTo(new Vector2(180,60),new Vector2(15,60))
                .lineTo(new Vector2(180,125.38))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(180,125.38),Rotation2.ZERO)
                .lineTo(new Vector2(180,67.04))
                .arcTo(new Vector2(225,22.04),new Vector2(225,67.04))
                .arcTo(new Vector2(270,67.04), new Vector2(225,67.04))
                .lineTo(new Vector2(270,125.38))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFour = new Trajectory(
                new SimplePathBuilder(new Vector2(270,125.38),Rotation2.ZERO)
                .lineTo(new Vector2(270,120))
                .arcTo(new Vector2(300,90), new Vector2(300,120))
                .lineTo(new Vector2(330,90))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        slalomPathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(30,30),Rotation2.ZERO)
                .lineTo(new Vector2(60,30))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        slalomPathPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(60,30),Rotation2.ZERO)
                .arcTo(new Vector2(90,60),new Vector2(60,60))
                .arcTo(new Vector2(120,90),new Vector2(120,60))
                .lineTo(new Vector2(240,90))
                .arcTo(new Vector2(270,60),new Vector2(240,60))
                .arcTo(new Vector2(315,34.02),new Vector2(300,60))
                .arcTo(new Vector2(315,85.98),new Vector2(300,60))
                .arcTo(new Vector2(270,60),new Vector2(300,60))
                .arcTo(new Vector2(240,30),new Vector2(240,60))
                .lineTo(new Vector2(120,30))
                .arcTo(new Vector2(90,60),new Vector2(120,60))
                .arcTo(new Vector2(60,90),new Vector2(60,60))
                .lineTo(new Vector2(30,90))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

    }

    public Trajectory getBarrelRacingAutoNavPartOne(){
        return barrelRacingAutoNavPartOne;
    }

    public Trajectory getBarrelRacingAutoNavPartTwo(){
        return barrelRacingAutoNavPartTwo;
    }

    public Trajectory getBouncePathPartOne() {
        return bouncePathPartOne;
    }

    public Trajectory getBouncePathPartTwo() {
        return bouncePathPartTwo;
    }

    public Trajectory getBouncePathPartThree() {
        return bouncePathPartThree;
    }

    public Trajectory getBouncePathPartFour() {
        return bouncePathPartFour;
    }

    public Trajectory getSlalomPathPartOne() {
        return slalomPathPartOne;
    }

    public Trajectory getSlalomPathPartTwo() {
        return slalomPathPartTwo;
    }
}
