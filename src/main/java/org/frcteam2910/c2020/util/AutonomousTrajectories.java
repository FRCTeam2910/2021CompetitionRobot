package org.frcteam2910.c2020.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.io.PathReader;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.*;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private static final String EIGHT_BALL_AUTO_PART_ONE_NAME = "autos/8BallAuto/8BallAutoPart1.path";
    private static final String EIGHT_BALL_AUTO_PART_TWO_NAME = "autos/8BallAuto/8BallAutoPart2.path";
    private static final String TEN_BALL_AUTO_PART_ONE_NAME = "autos/10BallAuto/10BallAutoPart1.path";
    private static final String TEN_BALL_AUTO_PART_TWO_NAME = "autos/10BallAuto/10BallAutoPart2.path";

    private Trajectory eightBallAutoPartOne;
    private Trajectory eightBallAutoPartTwo;
    private Trajectory eightBallAutoPartThree;
    private Trajectory tenBallAutoPartOne;
    private Trajectory tenBallAutoPartTwo;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 2);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        eightBallAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(509.0, -160.5), Rotation2.ZERO)
                .lineTo(new Vector2(455.8, -76.0))
                .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        eightBallAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(455.8, -76.0), Rotation2.ZERO)
                .arcTo(new Vector2(415.82, -134.25), new Vector2(415.82, -88.05))
                .lineTo(new Vector2(243.82, -134.25), Rotation2.fromDegrees(10.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        eightBallAutoPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(243.82, -134.25), Rotation2.fromDegrees(10.0))
                .lineTo(new Vector2(327.13, -134.25))
                .arcTo(new Vector2(455.8, -76.0), new Vector2(327.13, 51.53))
                .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        tenBallAutoPartOne  = new Trajectory(getPath(TEN_BALL_AUTO_PART_ONE_NAME), trajectoryConstraints, SAMPLE_DISTANCE);
        tenBallAutoPartTwo = new Trajectory(getPath(TEN_BALL_AUTO_PART_TWO_NAME), trajectoryConstraints, SAMPLE_DISTANCE);
    }

    private Path getPath(String name) throws IOException {
        InputStream in = getClass().getClassLoader().getResourceAsStream(name);
        if (in == null){
            throw new FileNotFoundException("Path file not found: " + name);
        }

        try (PathReader reader = new PathReader(new InputStreamReader(in))) {
            return reader.read();
        }
    }

    public Trajectory getEightBallAutoPartOne(){
        return eightBallAutoPartOne;
    }

    public Trajectory getEightBallAutoPartTwo(){
        return eightBallAutoPartTwo;
    }

    public Trajectory getEightBallAutoPartThree() {
        return eightBallAutoPartThree;
    }

    public Trajectory getTenBallAutoPartOne(){
        return tenBallAutoPartOne;
    }

    public Trajectory getTenBallAutoPartTwo(){
        return tenBallAutoPartTwo;
    }
}
