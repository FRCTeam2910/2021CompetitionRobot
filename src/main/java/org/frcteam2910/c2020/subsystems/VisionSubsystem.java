package org.frcteam2910.c2020.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;

import java.util.OptionalDouble;

public class VisionSubsystem implements Subsystem {
    private static final double TARGET_HEIGHT = 46.25;
    private static final double LIMELIGHT_HEIGHT = 5.5;

    private static final double INNER_TARGET_DEPTH = 29.25;

    private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(32.2);
    private static final double INNER_TARGET_RANGE_ANGLE = Math.toRadians(16.2);

    private static final Limelight LIMELIGHT = new Limelight();

    private final NetworkTableEntry distanceToTargetEntry;
    private final NetworkTableEntry canSeeInnerTargetEntry;

    private OptionalDouble distanceToTarget;
    private OptionalDouble angleToTarget;
    private boolean canSeeInnerTarget;

    public VisionSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        distanceToTargetEntry = tab.add("distance to target", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        canSeeInnerTargetEntry = tab.add("can see inner target", false)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }

    public OptionalDouble getAngleToTarget() {
        return angleToTarget;
    }

    public boolean isInnerTargetVisible() {
        return canSeeInnerTarget;
    }

    @Override
    public void periodic() {
        // Determine whether the Limelight has a target or not
        if (LIMELIGHT.hasTarget()) {
            // Calculate the distance to the outer target
            Vector2 targetPosition = LIMELIGHT.getTargetPosition();
            double theta = LIMELIGHT_MOUNTING_ANGLE + targetPosition.y;
            double distanceToOuterTarget = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /  Math.tan(theta);
            double dYOuter = distanceToOuterTarget * Math.sin(targetPosition.x);
            double dXOuter = Math.sqrt(Math.pow(distanceToOuterTarget, 2) - Math.pow(dYOuter, 2));

            // Calculate the distance to the inner target
            double dXInner = dXOuter + INNER_TARGET_DEPTH;
            double distanceToInnerTarget = Math.hypot(dXInner, dYOuter);
            double alpha = Math.asin(dYOuter / dXInner);

            // Bring to light whether we can see the inner target
            canSeeInnerTarget = MathUtils.isInRange(-INNER_TARGET_RANGE_ANGLE, INNER_TARGET_RANGE_ANGLE, alpha);
            if (canSeeInnerTarget) {
                distanceToTarget = OptionalDouble.of(distanceToInnerTarget);
                angleToTarget = OptionalDouble.of(alpha);
            } else {
                distanceToTarget = OptionalDouble.of(distanceToOuterTarget);
                angleToTarget = OptionalDouble.of(targetPosition.x);
            }
        } else {
            distanceToTarget = OptionalDouble.empty();
            angleToTarget = OptionalDouble.empty();
            canSeeInnerTarget = false;

            // Update shuffleboard
            distanceToTargetEntry.setDouble(-1.0);
            canSeeInnerTargetEntry.setBoolean(canSeeInnerTarget);
        }
    }
}
