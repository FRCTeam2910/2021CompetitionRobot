package org.frcteam2910.c2020.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.io.PathReader;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private static final String EIGHT_BALL_AUTO_PART_ONE_NAME = "autos/8BallAuto/8BallAutoPart1.path";
    private static final String EIGHT_BALL_AUTO_PART_TWO_NAME = "autos/8BallAuto/8BallAutoPart2.path";
    private static final String TEN_BALL_AUTO_PART_ONE_NAME = "autos/10BallAuto/10BallAutoPart1.path";
    private static final String TEN_BALL_AUTO_PART_TWO_NAME = "autos/10BallAuto/10BallAutoPart2.path";

    private static final Path EIGHT_BALL_COMPATIBLE_PART_ONE = new SimplePathBuilder(new Vector2(511.75, -148.0), Rotation2.ZERO)
            .lineTo(new Vector2(475.75, -134.25), Rotation2.fromDegrees(18.83))
            .build();
    private static final Path EIGHT_BALL_COMPATIBLE_PART_TWO = new SimplePathBuilder(new Vector2(475.75, -134.25), Rotation2.ZERO)
            .lineTo(new Vector2(324.0, -134.25), Rotation2.ZERO)
            .build();
    private static final Path EIGHT_BALL_COMPATIBLE_PART_THREE = new SimplePathBuilder(new Vector2(324.0, -134.25), Rotation2.ZERO)
            .lineTo(new Vector2(474.0, -114.25), Rotation2.fromDegrees(14.0))
            .build();
    private static final Path EIGHT_BALL_COMPATIBLE_PART_FOUR = new SimplePathBuilder(new Vector2(474.0, -114.25), Rotation2.fromDegrees(14.0))
            .lineTo(new Vector2(324.0,-134.25), Rotation2.fromDegrees(0.0))
            .build();

    private Trajectory eightBallAutoPartOne;
    private Trajectory eightBallAutoPartTwo;
    private Trajectory eightBallAutoPartThree;
    private Trajectory eightBallAutoPartFour;
    private Trajectory tenBallAutoPartOne;
    private Trajectory tenBallAutoPartTwo;
    private Trajectory circuitTenBallAutoPartOne;
    private Trajectory circuitTenBallAutoPartTwo;

    private final Trajectory eightBallCompatiblePartOne;
    private final Trajectory eightBallCompatiblePartTwo;
    private final Trajectory eightBallCompatiblePartThree;
    private final Trajectory eightBallCompatiblePartFour;

    private final Trajectory simpleShootThree;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        eightBallAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(509.0, -155.0), Rotation2.ZERO)
                        .lineTo(new Vector2(468.0, -67.34))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        eightBallAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(468.0, -67.34), Rotation2.ZERO)
                        .lineTo(new Vector2(459.23, -111.87))
                        .arcTo(new Vector2(432.0, -134.25), new Vector2(432.0, -106.5))
                        .lineTo(new Vector2(324.0, -134.25), Rotation2.fromDegrees(0.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        eightBallAutoPartThree = new Trajectory(
                new SimplePathBuilder(new Vector2(324.0, -134.25), Rotation2.fromDegrees(0.0))
                        .arcTo(new Vector2(468.0, -67.34), new Vector2(324.0, 54.16))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        eightBallAutoPartFour = new Trajectory(
                new SimplePathBuilder(new Vector2(468.0, -67.34), Rotation2.fromDegrees(0.0))
                        .arcTo(new Vector2(324, -134.25), new Vector2(324.0, 54.16))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        tenBallAutoPartOne = new Trajectory(getPath(TEN_BALL_AUTO_PART_ONE_NAME), trajectoryConstraints, SAMPLE_DISTANCE);
        tenBallAutoPartTwo = new Trajectory(getPath(TEN_BALL_AUTO_PART_TWO_NAME), trajectoryConstraints, SAMPLE_DISTANCE);

        circuitTenBallAutoPartOne = new Trajectory(
                new SimplePathBuilder(new Vector2(509.0, -162.0), Rotation2.ZERO)
                        .lineTo(new Vector2(385.51, -99.31), Rotation2.fromDegrees(290.0))
                        .arcTo(new Vector2(385.55, -77.48), new Vector2(390.51, -88.41))
                        .lineTo(new Vector2(418.33, -62.60))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        circuitTenBallAutoPartTwo = new Trajectory(
                new SimplePathBuilder(new Vector2(418.33, -62.60), Rotation2.fromDegrees(290.0))
                        .lineTo(new Vector2(413.52, -66.18), Rotation2.fromDegrees(290.0))
                        .arcTo(new Vector2(435.87, -94.38), new Vector2(424.28, -80.61))
                        .lineTo(new Vector2(468.0, -67.34), Rotation2.ZERO)
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        eightBallCompatiblePartOne = new Trajectory(EIGHT_BALL_COMPATIBLE_PART_ONE, trajectoryConstraints, SAMPLE_DISTANCE);
        eightBallCompatiblePartTwo = new Trajectory(EIGHT_BALL_COMPATIBLE_PART_TWO, slowConstraints, SAMPLE_DISTANCE);
        eightBallCompatiblePartThree = new Trajectory(EIGHT_BALL_COMPATIBLE_PART_THREE, trajectoryConstraints, SAMPLE_DISTANCE);
        eightBallCompatiblePartFour = new Trajectory(EIGHT_BALL_COMPATIBLE_PART_FOUR, trajectoryConstraints, SAMPLE_DISTANCE);

        simpleShootThree = new Trajectory(
                new SimplePathBuilder(Vector2.ZERO, Rotation2.ZERO)
                        .lineTo(new Vector2(40.0, 0.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
    }

    private Path getPath(String name) throws IOException {
        InputStream in = getClass().getClassLoader().getResourceAsStream(name);
        if (in == null) {
            throw new FileNotFoundException("Path file not found: " + name);
        }

        try (PathReader reader = new PathReader(new InputStreamReader(in))) {
            return reader.read();
        }
    }

    public Trajectory getEightBallAutoPartOne() {
        return eightBallAutoPartOne;
    }

    public Trajectory getEightBallAutoPartTwo() {
        return eightBallAutoPartTwo;
    }

    public Trajectory getEightBallAutoPartThree() {
        return eightBallAutoPartThree;
    }

    public Trajectory getEightBallAutoPartFour() {
        return eightBallAutoPartFour;
    }

    public Trajectory getTenBallAutoPartOne() {
        return tenBallAutoPartOne;
    }

    public Trajectory getTenBallAutoPartTwo() {
        return tenBallAutoPartTwo;
    }

    public Trajectory getCircuitTenBallAutoPartOne() {
        return circuitTenBallAutoPartOne;
    }

    public Trajectory getCircuitTenBallAutoPartTwo() {
        return circuitTenBallAutoPartTwo;
    }

    public Trajectory getEightBallCompatiblePartOne() {
        return eightBallCompatiblePartOne;
    }

    public Trajectory getEightBallCompatiblePartTwo() {
        return eightBallCompatiblePartTwo;
    }

    public Trajectory getEightBallCompatiblePartThree() {
        return eightBallCompatiblePartThree;
    }

    public Trajectory getEightBallCompatiblePartFour() {
        return eightBallCompatiblePartFour;
    }

    public Trajectory getSimpleShootThree() { return simpleShootThree; }
}
