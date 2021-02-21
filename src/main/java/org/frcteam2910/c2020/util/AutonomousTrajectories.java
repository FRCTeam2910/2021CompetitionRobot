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
        barrelRacingConstraints[barrelRacingConstraints.length - 1] = new MaxVelocityConstraint(13.0 * 12.0);
        barrelRacingConstraints[barrelRacingConstraints.length - 2] = new MaxAccelerationConstraint(7.0 * 12.0);
        barrelRacingConstraints[barrelRacingConstraints.length - 3] = new CentripetalAccelerationConstraint(18.0 * 12);

        TrajectoryConstraint[] bouncePathConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        bouncePathConstraints[bouncePathConstraints.length - 1] = new MaxVelocityConstraint(13.0 * 12.0);
        bouncePathConstraints[bouncePathConstraints.length - 2] = new MaxAccelerationConstraint(7.0 * 12.0);
        bouncePathConstraints[bouncePathConstraints.length - 3] = new CentripetalAccelerationConstraint(18.0 * 12);

        TrajectoryConstraint[] slalomPathConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        slalomPathConstraints[slalomPathConstraints.length - 1] = new MaxVelocityConstraint(13.0 * 12.0);
        slalomPathConstraints[slalomPathConstraints.length - 2] = new MaxAccelerationConstraint(7.0 * 12.0);
        slalomPathConstraints[slalomPathConstraints.length - 3] = new CentripetalAccelerationConstraint(18.0 * 12);



        barrelRacingMK2Part1 = new Trajectory(
                new SimplePathBuilder(new Vector2(42.75,90),Rotation2.ZERO)//original 42.75 90
                        .lineTo(new Vector2(43,90))//The robot thinks that the auto path starts at this point, not the vector in the SimplePathBuilder
                        .build(),
                barrelRacingConstraints,SAMPLE_DISTANCE
        );

        barrelRacingMk2Part2 = new Trajectory(
                new SimplePathBuilder(new Vector2(43,90),Rotation2.ZERO)
                .lineTo(new Vector2(146.22,99.82))//linear line to circle 1
                .arcTo(new Vector2(180.64,40),new Vector2(150,60))//circle 1
                .arcTo(new Vector2(115.36,40),new Vector2(150,60))//circle 1
                .arcTo(new Vector2(159.68,98.81),new Vector2(150,60))//circle 1
                .lineTo(new Vector2(230.82,81.19))//line to circle 2
                .arcTo(new Vector2(259.67,154.81),new Vector2(240,120))//circle 2
                .arcTo(new Vector2(211.72,91.72), new Vector2(240,120))//circle 2
                .lineTo(new Vector2(271.72,31.72))//linear line to circle 3
                .arcTo(new Vector2(338.97,50.98), new Vector2(300,60))//circle 3
                .arcTo(new Vector2(287.03,97.84), new Vector2(300,60))//circle 3
                .arcTo(new Vector2(233.63,90.14), new Vector2(240,235))//arc back to starting pos
                .lineTo(new Vector2(-120,105.69))//back home
                .build(),
                barrelRacingConstraints,SAMPLE_DISTANCE
        );


        bouncePathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(30,90),Rotation2.ZERO)
                .lineTo(new Vector2(30.25,90))//do the strange path thing
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(30.25,90),Rotation2.ZERO)
                .lineTo(new Vector2(60,90))
                .arcTo(new Vector2(90,120),new Vector2(60,120))
                .lineTo(new Vector2(90,125.38))//touches the 7 inch thing
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(90,125.38),Rotation2.ZERO)
                .lineTo(new Vector2(122.34,48.38))
                .arcTo(new Vector2(150,30), new Vector2(15,60))
                .arcTo(new Vector2(180,60), new Vector2(15,60))
                .lineTo(new Vector2(180,125.38))
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFour = new Trajectory(
                new SimplePathBuilder(new Vector2(180,125.38),Rotation2.ZERO)
                .lineTo(new Vector2(180,67.04))
                .arcTo(new Vector2(225,22.04),new Vector2(225,67.04))
                .arcTo(new Vector2(270,67.04), new Vector2(225,67.04))
                .lineTo(new Vector2(270,125.38))
                .build(),
                bouncePathConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFive = new Trajectory(
                new SimplePathBuilder(new Vector2(270,125.38),Rotation2.ZERO)
                .lineTo(new Vector2(270,120))
                .arcTo(new Vector2(300,90),new Vector2(300,120))
                .lineTo(new Vector2(330,90))
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
