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
        barrelRacingConstraints[barrelRacingConstraints.length - 1] = new MaxVelocityConstraint(17.0 * 12.0);//17ft
        barrelRacingConstraints[barrelRacingConstraints.length - 2] = new MaxAccelerationConstraint(22.0 * 12.0);//16ft
        barrelRacingConstraints[barrelRacingConstraints.length - 3] = new CentripetalAccelerationConstraint(24.0 * 12);//25ft

        TrajectoryConstraint[] bouncePathConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        bouncePathConstraints[bouncePathConstraints.length - 1] = new MaxVelocityConstraint(5.0 * 12.0);
        bouncePathConstraints[bouncePathConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);
        bouncePathConstraints[bouncePathConstraints.length - 3] = new CentripetalAccelerationConstraint(5.0 * 12);


        TrajectoryConstraint[] slalomPathConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        slalomPathConstraints[slalomPathConstraints.length - 1] = new MaxVelocityConstraint(17.0 * 12.0);//17
        slalomPathConstraints[slalomPathConstraints.length - 2] = new MaxAccelerationConstraint(20.0 * 12.0);//20
        slalomPathConstraints[slalomPathConstraints.length - 3] = new CentripetalAccelerationConstraint(15.0 * 12);//15



        barrelRacingMK2Part1 = new Trajectory(
                new SimplePathBuilder(new Vector2(42.75,90),Rotation2.ZERO)//original 42.75 90
                        .lineTo(new Vector2(43,90))//The robot thinks that the auto path starts at this point, not the vector in the SimplePathBuilder
                        .build(),
                barrelRacingConstraints,SAMPLE_DISTANCE
        );

        double xOffsetBarrel = -18.0;

        barrelRacingMk2Part2 = new Trajectory(
                new SimplePathBuilder(new Vector2(43,90),Rotation2.ZERO)
                        .lineTo(new Vector2(150 + xOffsetBarrel,90))//linear line to circle 1
                        .arcTo(new Vector2(175.98 + xOffsetBarrel,45),new Vector2(150 + xOffsetBarrel,60))//circle 1
                        .arcTo(new Vector2(124.09 + xOffsetBarrel,45),new Vector2(150 + xOffsetBarrel,60))//circle 1
                        .arcTo(new Vector2(150 + xOffsetBarrel,90),new Vector2(150 + xOffsetBarrel,60))//circle 1
                        .lineTo(new Vector2(231.02 + xOffsetBarrel,82.37))//line to circle 2
                        .arcTo(new Vector2(270.76 + xOffsetBarrel,142.80),new Vector2(240 + xOffsetBarrel,120))//circle 2
                        .arcTo(new Vector2(205.06 + xOffsetBarrel,135.69), new Vector2(240 + xOffsetBarrel,120))//circle 2
                        .arcTo(new Vector2(211.99 + xOffsetBarrel,93.86), new Vector2(240 + xOffsetBarrel,120))//circle 2
                        .lineTo(new Vector2(264.11 + xOffsetBarrel,39.33))//linear line to circle 3
                        .arcTo(new Vector2(326.30 + xOffsetBarrel,60), new Vector2(291.92 + xOffsetBarrel,60))//circle 3
                        .arcTo(new Vector2(291.92 + xOffsetBarrel,94.38), new Vector2(291.92 + xOffsetBarrel,60))//circle 3
                        .lineTo(new Vector2(-24,110))//back home
                        .build(),
                barrelRacingConstraints,SAMPLE_DISTANCE
        );

        double xOffsetBounce = 0.0;
        double yOffsetBounce = 0.0;

        bouncePathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(48,90),Rotation2.ZERO)
                .lineTo(new Vector2(48.25,90))//do the strange path thing
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartTwo = new Trajectory(//first ball
                new SimplePathBuilder(new Vector2(48.25,90),Rotation2.ZERO)
                .lineTo(new Vector2(60,90))
                .arcTo(new Vector2(90,120),new Vector2(60,120))//arc to the ball
                .lineTo(new Vector2(90,125.38 + 12.0))//touches the 7 inch thing
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartThree = new Trajectory(//second ball
                new SimplePathBuilder(new Vector2(90,125.38 + 12.0),Rotation2.ZERO)
                .lineTo(new Vector2(122.34 + xOffsetBounce,48.38 + yOffsetBounce))//go down from ball 1
                .arcTo(new Vector2(150 + xOffsetBounce,30 + yOffsetBounce), new Vector2(150 + xOffsetBounce,60 + yOffsetBounce))//arc to ball 2
                .arcTo(new Vector2(180 + xOffsetBounce,60 + yOffsetBounce), new Vector2(150 + xOffsetBounce,60 + yOffsetBounce))//arc to ball 2
                .lineTo(new Vector2(180 + xOffsetBounce,125.38 + 18.0))//travel upwards to ball 2
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFour = new Trajectory(//third ball
                new SimplePathBuilder(new Vector2(180 + xOffsetBounce,125.38 + 18.0),Rotation2.ZERO)
                .lineTo(new Vector2(180 + xOffsetBounce,67.04))//travel down to postion the robot for arc to the third ball
                .arcTo(new Vector2(255 + xOffsetBounce,22.04),new Vector2(225 + xOffsetBounce,67.04))//22.04 center y 67.04
                .arcTo(new Vector2(270 + xOffsetBounce,67.04),new Vector2(225 + xOffsetBounce,67.04))//67.04 center y67.04
                .lineTo(new Vector2(270 + xOffsetBounce,125.38 + 48.0))
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFive = new Trajectory(//home
                new SimplePathBuilder(new Vector2(270 + xOffsetBounce,125.38 + 48.0),Rotation2.ZERO)
                .lineTo(new Vector2(270 + xOffsetBounce,120 + 24))
                .arcTo(new Vector2(300 + xOffsetBounce,90),new Vector2(300,120))
                .lineTo(new Vector2(330 + xOffsetBounce,90))
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        double yOffsetSlalom = 0;
        double xOffsetSlalom = -18;

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
                .lineTo(new Vector2(250 + xOffsetSlalom,90))//go forwards
                .arcTo(new Vector2(278.87 + xOffsetSlalom,60), new Vector2(240 + xOffsetSlalom,60))//arc down
                .arcTo(new Vector2(310.56 + xOffsetSlalom,41.70),new Vector2(300 + xOffsetSlalom,60))////circle 1
                .arcTo(new Vector2(310.56 + xOffsetSlalom,78.29),new Vector2(300 + xOffsetSlalom,60))//circle 1
                .arcTo(new Vector2(278.87 + xOffsetSlalom,60), new Vector2(300 + xOffsetSlalom,60))//circle 1
                .arcTo(new Vector2(250 + xOffsetSlalom,30), new Vector2(240 + xOffsetSlalom,60))//circle 1
                .lineTo(new Vector2(120 + xOffsetSlalom + 6,30))//go forwards to home

                .arcTo(new Vector2(90 + xOffsetSlalom + 6,60),new Vector2(120 + xOffsetSlalom + 6,60))

                .lineTo(new Vector2(90 + xOffsetSlalom + 6,72))
                .arcTo(new Vector2(68.37 + xOffsetSlalom + 6,93.63),new Vector2(68.37 + xOffsetSlalom + 6,72))
                .lineTo(new Vector2(30 + xOffsetSlalom - 6,90 + 6))
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
