package org.frcteam2910.c2020.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.IOException;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory barrelRacingMK2Part1;
    private Trajectory barrelRacingMk2Part2;

    private Trajectory bouncePathPartOne;
    private Trajectory bouncePathPartTwo;
    private Trajectory bouncePathPartThree;
    private Trajectory bouncePathPartFour;
    private Trajectory bouncePathPartFive;

    private Trajectory slalomPathPartOne;
    private Trajectory slalomPathPartTwo;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] barrelRacingConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        barrelRacingConstraints[barrelRacingConstraints.length - 1] = new MaxVelocityConstraint(17.0 * 12.0);
        barrelRacingConstraints[barrelRacingConstraints.length - 2] = new MaxAccelerationConstraint(22.0 * 12.0);
        barrelRacingConstraints[barrelRacingConstraints.length - 3] = new CentripetalAccelerationConstraint(24.0 * 12);

        TrajectoryConstraint[] bouncePathConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        bouncePathConstraints[bouncePathConstraints.length - 1] = new MaxVelocityConstraint(17 * 12.0);
        bouncePathConstraints[bouncePathConstraints.length - 2] = new MaxAccelerationConstraint(20 * 12.0);
        bouncePathConstraints[bouncePathConstraints.length - 3] = new CentripetalAccelerationConstraint(25.0 * 12);


        TrajectoryConstraint[] slalomPathConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        slalomPathConstraints[slalomPathConstraints.length - 1] = new MaxVelocityConstraint(17.0 * 12.0);
        slalomPathConstraints[slalomPathConstraints.length - 2] = new MaxAccelerationConstraint(20.0 * 12.0);
        slalomPathConstraints[slalomPathConstraints.length - 3] = new CentripetalAccelerationConstraint(17.0 * 12);



        barrelRacingMK2Part1 = new Trajectory(
                new SimplePathBuilder(new Vector2(42.75,90),Rotation2.ZERO)
                        .lineTo(new Vector2(43,90))//The robot thinks that the auto path starts at this point, not the vector in the SimplePathBuilder
                        .build(),
                barrelRacingConstraints,SAMPLE_DISTANCE
        );



        barrelRacingMk2Part2 = new Trajectory(
                new SimplePathBuilder(new Vector2(43,90),Rotation2.ZERO)
                        .lineTo(new Vector2(150 - 18,90))//linear line to circle 1
                        .arcTo(new Vector2(175.98 - 18,45),new Vector2(150 - 18,60))//circle 1
                        .arcTo(new Vector2(124.09 - 18,45),new Vector2(150 - 18,60))//circle 1
                        .arcTo(new Vector2(150 - 18,90),new Vector2(150 - 18,60))//circle 1
                        .lineTo(new Vector2(231.02 - 18,82.37))//line to circle 2
                        .arcTo(new Vector2(270.76 - 18,142.80),new Vector2(240 - 18,120))//circle 2
                        .arcTo(new Vector2(205.06 - 18,135.69), new Vector2(240 - 18,120))//circle 2
                        .arcTo(new Vector2(211.99 - 18,93.86), new Vector2(240 - 18,120))//circle 2
                        .lineTo(new Vector2(264.11 - 18,39.33))//linear line to circle 3
                        .arcTo(new Vector2(326.30 - 18,60), new Vector2(291.92 - 18,60))//circle 3
                        .arcTo(new Vector2(291.92 - 18,94.38), new Vector2(291.92 - 18,60))//circle 3
                        .lineTo(new Vector2(-24,110))//back home
                        .build(),
                barrelRacingConstraints,SAMPLE_DISTANCE
        );


        bouncePathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(43,90),Rotation2.ZERO)
                .lineTo(new Vector2(43.25,90))//do the strange path thing
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartTwo = new Trajectory(//first ball
                new SimplePathBuilder(new Vector2(43.25,90),Rotation2.ZERO)
                .lineTo(new Vector2(60 - 12,90))
                .arcTo(new Vector2(90 - 12 ,120),new Vector2(60 - 12,120))//arc to the ball
                .lineTo(new Vector2(90 - 12,125.38 + 17))//touches the 7 inch thing
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartThree = new Trajectory(//second ball
                new SimplePathBuilder(new Vector2(90 - 12,125.38 + 17),Rotation2.ZERO)
                .lineTo(new Vector2(122.34 - 22,48.38 + 24.0 + 6 ))//go down from ball 1
                .arcTo(new Vector2(150 - 22,30 + 24.0 + 6), new Vector2(150 - 22,60 + 30))//arc to ball 2
                .arcTo(new Vector2(180 - 22,60 + 24.0 + 6), new Vector2(150 - 22,60 + 30))//arc to ball 2
                .lineTo(new Vector2(180 - 16,125.38 + 59))//travel upwards to ball 2
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFour = new Trajectory(//third ball
                new SimplePathBuilder(new Vector2(180 - 16,125.38 + 59),Rotation2.ZERO)
                .lineTo(new Vector2(180 - 18,67.04 + 80))//travel down to postion the robot for arc to the third ball
                .arcTo(new Vector2(255 - 18,22.04 + 82),new Vector2(225 - 17,67.04 + 82))//22.04 center y 67.04
                .arcTo(new Vector2(270 - 18,67.04 + 82),new Vector2(225 - 17,67.04 + 82))//67.04 center y67.04
                .lineTo(new Vector2(270 - 18,125.38 + 116))
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFive = new Trajectory(//home
                new SimplePathBuilder(new Vector2(270 - 18,125.38 + 116),Rotation2.ZERO)//241
                .lineTo(new Vector2(270 - 13,120 + 114))//
                .arcTo(new Vector2(300  - 13 ,90 + 116),new Vector2(300 - 13,120 + 116))
                .lineTo(new Vector2(330 - 1 ,90 + 116))
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );


        slalomPathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(40,30),Rotation2.ZERO)
                .lineTo(new Vector2(40.25,30))
                .build(),
                slalomPathConstraints,SAMPLE_DISTANCE
        );

        slalomPathPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(40.25,30),Rotation2.ZERO)
                .lineTo(new Vector2(60 - 10,30))//exit starting zone
                .arcTo(new Vector2(90 - 10,60),new Vector2(60 - 10,60))//arc left
                .arcTo(new Vector2(120 - 10,90),new Vector2(120 - 10,60))//arc right
                .lineTo(new Vector2(250 - 18,90))//go forwards
                .arcTo(new Vector2(278.87 - 18,60), new Vector2(240 - 18,60))//arc down
                .arcTo(new Vector2(310.56 - 18,41.70),new Vector2(300 - 18,60))////circle 1
                .arcTo(new Vector2(310.56 - 18,78.29),new Vector2(300 - 18,60))//circle 1
                .arcTo(new Vector2(278.87 - 18,60), new Vector2(300 - 18,60))//circle 1
                .arcTo(new Vector2(250 - 18,30), new Vector2(240 - 18,60))//circle 1
                .lineTo(new Vector2(120 - 12,30))//go forwards to home
                .arcTo(new Vector2(90 - 12,60),new Vector2(120 - 12,60))//arc up
                .lineTo(new Vector2(90 - 12,72))//go up
                .arcTo(new Vector2(68.37 - 12,93.63),new Vector2(68.37 - 12,72))//arc left
                .lineTo(new Vector2(30 - 24,90 + 6))//line to home
                .build(),
                slalomPathConstraints,SAMPLE_DISTANCE
        );

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

    public Trajectory getBouncePathPartFive(){
        return bouncePathPartFive;
    }

    public Trajectory getSlalomPathPartOne() {
        return slalomPathPartOne;
    }

    public Trajectory getSlalomPathPartTwo() {
        return slalomPathPartTwo;
    }

    public Trajectory getBarrelRacingMK2Part1() {
        return barrelRacingMK2Part1;
    }

    public Trajectory getBarrelRacingMk2Part2() {
        return barrelRacingMk2Part2;
    }
}
