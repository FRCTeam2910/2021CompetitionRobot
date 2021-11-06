package org.frcteam2910.c2020.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.IOException;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory optimizedBarrelRacing;

    private Trajectory optimizedSlalom;

    private Trajectory bouncePathMk2;

    //Galactic search
    private Trajectory pathARed;
    private Trajectory pathABlue;
    private Trajectory pathBRed;
    private Trajectory pathBBlue;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {

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