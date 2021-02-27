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
        slalomPathConstraints[slalomPathConstraints.length - 1] = new MaxVelocityConstraint(5.0 * 12.0);
        slalomPathConstraints[slalomPathConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);
        slalomPathConstraints[slalomPathConstraints.length - 3] = new CentripetalAccelerationConstraint(5.0 * 12);



        barrelRacingMK2Part1 = new Trajectory(
                new SimplePathBuilder(new Vector2(42.75,90),Rotation2.ZERO)//original 42.75 90
                        .lineTo(new Vector2(43,90))//The robot thinks that the auto path starts at this point, not the vector in the SimplePathBuilder
                        .build(),
                barrelRacingConstraints,SAMPLE_DISTANCE
        );

        double xOffset = -18.0;

        barrelRacingMk2Part2 = new Trajectory(
                new SimplePathBuilder(new Vector2(43,90),Rotation2.ZERO)
                        .lineTo(new Vector2(150 + xOffset,90))//linear line to circle 1
                        .arcTo(new Vector2(175.98 + xOffset,45),new Vector2(150 + xOffset,60))//circle 1
                        .arcTo(new Vector2(124.09 + xOffset,45),new Vector2(150 + xOffset,60))//circle 1
                        .arcTo(new Vector2(150 + xOffset,90),new Vector2(150 + xOffset,60))//circle 1
                        .lineTo(new Vector2(231.02 + xOffset,82.37))//line to circle 2
                        .arcTo(new Vector2(270.76 + xOffset,142.80),new Vector2(240 + xOffset,120))//circle 2
                        .arcTo(new Vector2(205.06 + xOffset,135.69), new Vector2(240 + xOffset,120))//circle 2
                        .arcTo(new Vector2(211.99 + xOffset,93.86), new Vector2(240 + xOffset,120))//circle 2
                        .lineTo(new Vector2(264.11 + xOffset,39.33))//linear line to circle 3
                        .arcTo(new Vector2(326.30 + xOffset,60), new Vector2(291.92 + xOffset,60))//circle 3
                        .arcTo(new Vector2(291.92 + xOffset,94.38), new Vector2(291.92 + xOffset,60))//circle 3
                        .lineTo(new Vector2(-24,110))//back home
                        .build(),
                barrelRacingConstraints,SAMPLE_DISTANCE
        );


//        barrelRacingMk2Part2 = new Trajectory(
//                new SimplePathBuilder(new Vector2(43,90),Rotation2.ZERO)
//                .lineTo(new Vector2(146.22,99.82))//linear line to circle 1
//                .arcTo(new Vector2(180.64,40),new Vector2(150,60))//circle 1
//                .arcTo(new Vector2(115.36,40),new Vector2(150,60))//circle 1
//                .arcTo(new Vector2(159.68,98.81),new Vector2(150,60))//circle 1
//                .lineTo(new Vector2(230.91,81.17))//line to circle 2
//                .arcTo(new Vector2(272.03,143.73),new Vector2(240,120))//circle 2
//                .arcTo(new Vector2(203.63,136.34), new Vector2(240,120))//circle 2
//                .arcTo(new Vector2(210.86,92.78), new Vector2(240,120))//circle 2
//                .lineTo(new Vector2(262.35,37.57))//linear line to circle 3
//                .arcTo(new Vector2(328.84,60), new Vector2(291.92,60))//circle 3
//                .arcTo(new Vector2(291.92,96.91), new Vector2(291.92,60))//circle 3
//                .lineTo(new Vector2(-24,102))//back home
//                .build(),
//                barrelRacingConstraints,SAMPLE_DISTANCE
//        );


        bouncePathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(30,90),Rotation2.ZERO)
                .lineTo(new Vector2(30.25,90))//do the strange path thing
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartTwo = new Trajectory(//first ball
                new SimplePathBuilder(new Vector2(30.25,90),Rotation2.ZERO)
                .lineTo(new Vector2(60,90))
                .arcTo(new Vector2(90,120),new Vector2(60,120))//arc to the ball
                .lineTo(new Vector2(90,137.38))//touches the 7 inch thing
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartThree = new Trajectory(//second ball
                new SimplePathBuilder(new Vector2(90,137.38),Rotation2.ZERO)
                .lineTo(new Vector2(102.34,60.38))//go down from ball 1
                .arcTo(new Vector2(132,42), new Vector2(132,72))//arc to ball 2
                .arcTo(new Vector2(172,72), new Vector2(132,72))//arc to ball 2
                .lineTo(new Vector2(172,155.38))//travel upwards to ball 2
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFour = new Trajectory(//third ball
                new SimplePathBuilder(new Vector2(172,155.38),Rotation2.ZERO)
                .lineTo(new Vector2(172,103.04))//travel down to postion the robot for arc to the third ball
                .arcTo(new Vector2(255,58.04),new Vector2(225,103.04))//22.04 center y 67.04
                .arcTo(new Vector2(270,103.04),new Vector2(225,103.04))//67.04 center y67.04
                .lineTo(new Vector2(270,161.38))
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFive = new Trajectory(//home
                new SimplePathBuilder(new Vector2(197,125.38),Rotation2.ZERO)
                .lineTo(new Vector2(252,120))
                .arcTo(new Vector2(270,102),new Vector2(300,132))
                .lineTo(new Vector2(300,9102))
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        slalomPathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(30,30),Rotation2.ZERO)
                .lineTo(new Vector2(30.25,30))
                .build(),
                slalomPathConstraints,SAMPLE_DISTANCE
        );

        slalomPathPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(30.25,30),Rotation2.ZERO)
                .lineTo(new Vector2(60,30))
                .arcTo(new Vector2(90,60),new Vector2(60,60))
                .arcTo(new Vector2(120,90),new Vector2(120,60))
                .lineTo(new Vector2(240,90))
                .arcTo(new Vector2(270,60), new Vector2(240,60))
                .arcTo(new Vector2(315,34.02),new Vector2(300,60))
                .arcTo(new Vector2(315,85.98),new Vector2(300,60))
                .arcTo(new Vector2(270,60), new Vector2(300,60))
                .arcTo(new Vector2(240,30), new Vector2(240,60))
                .lineTo(new Vector2(120,30))
                .arcTo(new Vector2(90,60), new Vector2(120,60))
                .arcTo(new Vector2(60,90), new Vector2(60,60))
                .lineTo(new Vector2(30,90))
                .build(),
                slalomPathConstraints,SAMPLE_DISTANCE
        );

//        slalomPathPartOne = new Trajectory(
//                new SimplePathBuilder(new Vector2(30,30),Rotation2.ZERO)
//                .lineTo(new Vector2(60,30))
//                .build(),
//                trajectoryConstraints,SAMPLE_DISTANCE
//        );
//
//        slalomPathPartTwo = new Trajectory(
//                new SimplePathBuilder(new Vector2(60,30),Rotation2.ZERO)
//                .arcTo(new Vector2(90,60),new Vector2(60,60))
//                .arcTo(new Vector2(120,90),new Vector2(120,60))
//                .lineTo(new Vector2(240,90))
//                .arcTo(new Vector2(270,60),new Vector2(240,60))
//                .arcTo(new Vector2(315,34.02),new Vector2(300,60))
//                .arcTo(new Vector2(315,85.98),new Vector2(300,60))
//                .arcTo(new Vector2(270,60),new Vector2(300,60))
//                .arcTo(new Vector2(240,30),new Vector2(240,60))
//                .lineTo(new Vector2(120,30))
//                .arcTo(new Vector2(90,60),new Vector2(120,60))
//                .arcTo(new Vector2(60,90),new Vector2(60,60))
//                .lineTo(new Vector2(30,90))
//                .build(),
//                trajectoryConstraints,SAMPLE_DISTANCE
//        );

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
