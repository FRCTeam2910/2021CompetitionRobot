package org.frcteam2910.c2020.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.IOException;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory barrelRacingMK2Part1;
    private Trajectory barrelRacingMk2Part2;

    private Trajectory optimizedBarrelRacing;

    private Trajectory bouncePathPartOne;
    private Trajectory bouncePathPartTwo;
    private Trajectory bouncePathPartThree;
    private Trajectory bouncePathPartFour;
    private Trajectory bouncePathPartFive;

    private Trajectory slalomPathPartOne;
    private Trajectory slalomPathPartTwo;

    private Trajectory optimizedSlalom;

    private Trajectory bouncePathMk2;

    private Trajectory pathARed;
    private Trajectory pathABlue;
    private Trajectory pathBRed;
    private Trajectory pathBBlue;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {


        barrelRacingMK2Part1 = new Trajectory(
                new SimplePathBuilder(new Vector2(42.75,90),Rotation2.ZERO)
                        .lineTo(new Vector2(43,90))//The robot thinks that the auto path starts at this point, not the vector in the SimplePathBuilder
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );


        optimizedBarrelRacing = new Trajectory(
                new SimplePathBuilder(new Vector2(45.75,104.25),Rotation2.ZERO)
                .lineTo(new Vector2(154.422,85.114),Rotation2.fromDegrees(30))
                .arcTo(new Vector2(175.5,60), new Vector2(150,60),Rotation2.ZERO)
                .arcTo(new Vector2(150,34.5),new Vector2(150,60),Rotation2.fromDegrees(-90))
                .arcTo(new Vector2(124.5,60), new Vector2(150,60),Rotation2.fromDegrees(180))
                .arcTo(new Vector2(145.983,85.182), new Vector2(150,60),Rotation2.fromDegrees(99.06))
                .lineTo(new Vector2(244.017,100.818),Rotation2.fromDegrees(99.06))
                .arcTo(new Vector2(265.5,120), new Vector2(240,126),Rotation2.fromDegrees(180))
                .arcTo(new Vector2(240,151.5), new Vector2(240,126),Rotation2.fromDegrees(-90))
                .arcTo(new Vector2(214.5,126), new Vector2(240,120),Rotation2.ZERO)
                .arcTo(new Vector2(297.577,34.615), new Vector2(306.3,126),Rotation2.ZERO)
                .arcTo(new Vector2(312.722,82.099),new Vector2(300,60),Rotation2.fromDegrees(150.07))
                .arcTo(new Vector2(286.262,90), new Vector2(282.787,30.101),Rotation2.fromDegrees(180))
                .lineTo(new Vector2(-24,108),Rotation2.fromDegrees(180))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE

        );


        optimizedSlalom = new Trajectory(
                new SimplePathBuilder(new Vector2(45.75,15.75),Rotation2.ZERO)
                .lineTo(new Vector2(90,60),Rotation2.fromDegrees(45))
                .lineTo(new Vector2(105,75),Rotation2.fromDegrees(45))
                .arcTo(new Vector2(122.237,87.03),new Vector2(147.426,32.574),Rotation2.fromDegrees(45))
                .arcTo(new Vector2(243.117,89.261),new Vector2(185.21,-49.111),Rotation2.fromDegrees(31.79))
                .arcTo(new Vector2(261.675,73.433),new Vector2(277.675,52.362),Rotation2.fromDegrees(31.79))
                .lineTo(new Vector2(278.325,46.567),Rotation2.fromDegrees(31.79))
                .arcTo(new Vector2(325.5,60),new Vector2(300,60),Rotation2.fromDegrees(180))
                .arcTo(new Vector2(278.325,73.325),new Vector2(300,60),Rotation2.fromDegrees(-31.79))
                .lineTo(new Vector2(261.675,46.567),Rotation2.fromDegrees(-31.79))
                .arcTo(new Vector2(238.763,26.903),new Vector2(219.175,72.906),Rotation2.fromDegrees(-58.21))
                .arcTo(new Vector2(121.238,26.903),new Vector2(180,164.913),Rotation2.fromDegrees(-58.21))
                .arcTo(new Vector2(98.325,46.567),new Vector2(140.825,72.906),Rotation2.fromDegrees(-58.21))
                .lineTo(new Vector2(30,156.814),Rotation2.fromDegrees(58.21))
                .build()
                ,
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        bouncePathMk2 = new Trajectory(
                new SimplePathBuilder(new Vector2(44,90),Rotation2.ZERO)
                .arcTo(new Vector2(89.511,136.104),new Vector2(30.812,148.533),Rotation2.fromDegrees(24.22))
                .arcTo(new Vector2(90.461   ,136.193), new Vector2(90,136),Rotation2.fromDegrees(24.22))//touches ball small arc 1
                .lineTo(new Vector2(126.745,49.538),Rotation2.fromDegrees(24.22))//go down
                .arcTo(new Vector2(175.462,58.599),new Vector2(150,60),Rotation2.fromDegrees(176.85))//arc to middle ball
                .lineTo(new Vector2(179.501,142.024),Rotation2.fromDegrees(135))//ball touch
                .arcTo(new Vector2(180.5,141.996),new Vector2(180,142),Rotation2.fromDegrees(135))//ball touch arc
                .lineTo(new Vector2(180.002,72.376),Rotation2.fromDegrees(135))//go down from middle
                .arcTo(new Vector2(223.514,27.025),new Vector2(225,72),Rotation2.fromDegrees(135))//big arc to ball 3 part 1
                .arcTo(new Vector2(269.992,69.352),new Vector2(225,72),Rotation2.fromDegrees(135))//big arc to ball 3 part 2
                .lineTo(new Vector2(274.501,147.029),Rotation2.fromDegrees(135))//go to ball 3
                .arcTo(new Vector2(275.499,147.025),new Vector2(275,147),Rotation2.fromDegrees(135))//ball 3 arc
                .arcTo(new Vector2(335.426,90),new Vector2(335.426,150),Rotation2.fromDegrees(135))//arc home
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );



        barrelRacingMk2Part2 = new Trajectory(
                new SimplePathBuilder(new Vector2(43,90),Rotation2.ZERO)
                        .lineTo(new Vector2(150,90))//linear line to circle 1
                        .arcTo(new Vector2(175.98,45),new Vector2(150,60))//circle 1
                        .arcTo(new Vector2(124.09,45),new Vector2(150,60))//circle 1
                        .arcTo(new Vector2(150,90),new Vector2(150,60))//circle 1
                        .lineTo(new Vector2(231.02,82.37))//line to circle 2
                        .arcTo(new Vector2(270.76 ,142.80),new Vector2(240 ,120))//circle 2
                        .arcTo(new Vector2(205.06,135.69), new Vector2(240,120))//circle 2
                        .arcTo(new Vector2(211.99,93.86), new Vector2(240,120))//circle 2
                        .lineTo(new Vector2(264.11,39.33))//linear line to circle 3
                        .arcTo(new Vector2(326.30,60), new Vector2(291.92,60))//circle 3
                        .arcTo(new Vector2(291.92,94.38), new Vector2(291.92,60))//circle 3
                        .lineTo(new Vector2(0,110))//back home
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );


        bouncePathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(43,90),Rotation2.ZERO)
                        .lineTo(new Vector2(43.25,90))//do the strange path thing
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartTwo = new Trajectory(//first ball
                new SimplePathBuilder(new Vector2(43.25,90),Rotation2.ZERO)
                        .lineTo(new Vector2(60,90))
                        .arcTo(new Vector2(90 ,120),new Vector2(60 ,120))//arc to the ball
                        .lineTo(new Vector2(90,150))//touches the 7 inch thing
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartThree = new Trajectory(//second ball
                new SimplePathBuilder(new Vector2(90,150),Rotation2.ZERO)
                        .lineTo(new Vector2(122.34,48.38 ))//go down from ball 1
                        .arcTo(new Vector2(150,30 ), new Vector2(150 ,60))//arc to ball 2
                        .arcTo(new Vector2(180,60), new Vector2(150,60))//arc to ball 2
                        .lineTo(new Vector2(180 ,150))//travel upwards to ball 2
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFour = new Trajectory(//third ball
                new SimplePathBuilder(new Vector2(180,150),Rotation2.ZERO)
                        .lineTo(new Vector2(180,67.04))//travel down to postion the robot for arc to the third ball
                        .arcTo(new Vector2(255,22.04 ),new Vector2(225,67.04))//22.04 center y 67.04
                        .arcTo(new Vector2(270,67.04 ),new Vector2(225,67.04 ))//67.04 center y67.04
                        .lineTo(new Vector2(270 ,150))
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        bouncePathPartFive = new Trajectory(//home
                new SimplePathBuilder(new Vector2(270,150),Rotation2.ZERO)//241
                        .lineTo(new Vector2(270,120 ))//
                        .arcTo(new Vector2(300 ,90 ),new Vector2(300 ,120))
                        .lineTo(new Vector2(330  ,90 ))
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );


        slalomPathPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(40,30),Rotation2.ZERO)
                        .lineTo(new Vector2(40.25,30))
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        slalomPathPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(40.25,30),Rotation2.ZERO)
                        .lineTo(new Vector2(60 ,30))//exit starting zone
                        .arcTo(new Vector2(90 ,60),new Vector2(60 ,60))//arc left
                        .arcTo(new Vector2(120 ,90),new Vector2(120 ,60))//arc right
                        .lineTo(new Vector2(250 ,90))//go forwards
                        .arcTo(new Vector2(278.87 ,60), new Vector2(240 ,60))//arc down
                        .arcTo(new Vector2(310.56 ,41.70),new Vector2(300 ,60))////circle 1
                        .arcTo(new Vector2(310.56 ,78.29),new Vector2(300 ,60))//circle 1
                        .arcTo(new Vector2(278.87 ,60), new Vector2(300 ,60))//circle 1
                        .arcTo(new Vector2(250 ,30), new Vector2(240 ,60))//circle 1
                        .lineTo(new Vector2(120 ,30))//go forwards to home
                        .arcTo(new Vector2(90 ,60),new Vector2(120 ,60))//arc up
                        .lineTo(new Vector2(90 ,72))//go up
                        .arcTo(new Vector2(68.37 ,93.63),new Vector2(68.37 ,72))//arc left
                        .lineTo(new Vector2(30 ,90 ))//line to home
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );



        pathARed = new Trajectory(
                new SimplePathBuilder(new Vector2(44.25,90),Rotation2.fromDegrees(180))
                .arcTo(new Vector2(125.32,67),new Vector2(58.535,-14.024),Rotation2.fromDegrees(10 + 180))
                .arcTo(new Vector2(158,81.161),new Vector2(138.04,82.433),Rotation2.fromDegrees(25 + 180))
                .lineTo(new Vector2(160,112.544),Rotation2.fromDegrees(45 + 180))
                .arcTo(new Vector2(199.919,150),new Vector2(199.919,110),Rotation2.fromDegrees(45 + 180))
                .lineTo(new Vector2(360,150),Rotation2.fromDegrees(45 + 180))
                .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        pathABlue = new Trajectory(
                new SimplePathBuilder(new Vector2(44.25,30),Rotation2.fromDegrees(90))
                        .lineTo(new Vector2(165.25,30),Rotation2.fromDegrees(23.61 + 180))
                        .arcTo(new Vector2(199,63.75),new Vector2(165.25,63.75),Rotation2.fromDegrees(23.61 + 180))
                        .lineTo(new Vector2(199,96),Rotation2.fromDegrees(24 + 180))
                        .arcTo(new Vector2(242.565,112.744),new Vector2(224,96),Rotation2.fromDegrees(20.26 + 180))
                        .arcTo(new Vector2(307.269,86.5),new Vector2(301.971,166.324),Rotation2.fromDegrees(-22.73 + 180))
                        .lineTo(new Vector2(360,90),Rotation2.fromDegrees(-45 + 180))
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        pathBRed = new Trajectory(
                new SimplePathBuilder(new Vector2(48.153,150),Rotation2.fromDegrees(45))
                        .lineTo(new Vector2(133,75.022),Rotation2.fromDegrees(-45 + 180))
                        .arcTo(new Vector2(197.499,92.542),new Vector2(159.487,104.996),Rotation2.fromDegrees(-25 + 180))
                        .arcTo(new Vector2(243,130),new Vector2(249.765,75.417),Rotation2.fromDegrees(45 + 180))
                        .lineTo(new Vector2(360,144.501),Rotation2.fromDegrees(10 + 180))
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
        );

        pathBBlue = new Trajectory(
                new SimplePathBuilder(new Vector2(44.25,30),Rotation2.fromDegrees(-45))
                        .lineTo(new Vector2(172,56),Rotation2.fromDegrees(21.77 + 180))
                        .arcTo(new Vector2(209.663,90.072),new Vector2(161.936,104.977),Rotation2.fromDegrees(45 + 180))
                        .arcTo(new Vector2(271,102),new Vector2(243.549,79.49),Rotation2.fromDegrees(5.09 + 180))
                        .lineTo(new Vector2(354.642,0),Rotation2.fromDegrees(-50.49 + 180))
                        .build(),
                trajectoryConstraints,SAMPLE_DISTANCE
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

    public Trajectory getOptimizedBarrelRacing(){
        return optimizedBarrelRacing;
    }

    public Trajectory getOptimizedSlalom() {
        return optimizedSlalom;
    }

    public Trajectory getBouncePathMk2() {
        return bouncePathMk2;
    }


    public Trajectory getPathARed() {
        return pathARed;
    }

    public Trajectory getPathABlue() {
        return pathABlue;
    }

    public Trajectory getPathBRed() {
        return pathBRed;
    }

    public Trajectory getPathBBlue() {
        return pathBBlue;
    }
}