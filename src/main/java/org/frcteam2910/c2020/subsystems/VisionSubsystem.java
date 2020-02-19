package org.frcteam2910.c2020.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;

import java.util.OptionalDouble;

public class VisionSubsystem implements Subsystem {
    private static final double TARGET_HEIGHT = 98.25;
    private static final double LIMELIGHT_HEIGHT = 21.85;

    private static final double INNER_TARGET_RANGE_ANGLE = Math.toRadians(18.0);
    private static final double INNER_TARGET_DEPTH = 29.25;
    // The distance from the inner target to the apex of the triangle we use to find the distance
    private static final double DISTANCE_FROM_INNER_TO_APEX = 16.92;

    private static final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(32.0);

    private static final double TARGET_ALLOWABLE_ERROR = Math.toRadians(1.0);

    private static final Limelight LIMELIGHT = new Limelight();
    private final DrivetrainSubsystem drivetrain;

    private final NetworkTableEntry distanceToTargetEntry;
    private final NetworkTableEntry dXOuterEntry;
    private final NetworkTableEntry dYOuterEntry;
    private final NetworkTableEntry canSeeInnerTargetEntry;

    private boolean hasTarget;
    private boolean isInnerTargetVisible;
    private OptionalDouble distanceToTarget = OptionalDouble.empty();
    private OptionalDouble angleToTarget = OptionalDouble.empty();

    public VisionSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
        drivetrain = drivetrainSubsystem;
        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        distanceToTargetEntry = tab.add("distance to target", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        dXOuterEntry = tab.add("dXOuter", 0.0)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();
        dYOuterEntry = tab.add("dYOuter", 0.0)
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();
        canSeeInnerTargetEntry = tab.add("can see inner target", false)
                .withPosition(3, 0)
                .withSize(1, 1)
                .getEntry();
        tab.addNumber("tx", () -> Math.toDegrees(getAngleToTarget().orElse(Double.NaN)))
                .withPosition(4, 0)
                .withSize(1, 1);
    }

    public void setCamMode(Limelight.CamMode mode) {
        LIMELIGHT.setCamMode(mode);
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }

    public OptionalDouble getAngleToTarget() {
        return angleToTarget;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public boolean isInnerTargetVisible() {
        return isInnerTargetVisible;
    }

    @Override
    public void periodic() {
        // Determine whether the Limelight has a target or not
        hasTarget = LIMELIGHT.hasTarget();
        if (hasTarget) {
            // Calculate the distance to the outer target
            Vector2 targetPosition = LIMELIGHT.getTargetPosition();
            double theta = LIMELIGHT_MOUNTING_ANGLE + targetPosition.y;
            double distanceToOuterTarget = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /  Math.tan(theta);

            // Get the field oriented angle for the outer target, with latency compensation
            double tx = drivetrain.getPoseAtTime(Timer.getFPGATimestamp() - LIMELIGHT.getPipelineLatency() / 1000.0).rotation.toRadians() - targetPosition.x;
            double dYOuter = distanceToOuterTarget * Math.sin(tx);
            double dXOuter = distanceToOuterTarget * Math.cos(tx);

            // Calculate the distance to the inner target
            double dXInner = dXOuter + INNER_TARGET_DEPTH;
            double distanceToInnerTarget = Math.hypot(dXInner, dYOuter);
            // Add DISTANCE_FROM_INNER_TO_APEX to dXInner here because we want if we did it when we defined dXInner
            // distanceToInnerTarget would be incorrect, and we only need this extra bit to calculate the angle
            double alpha = Math.atan(dYOuter / (dXInner + DISTANCE_FROM_INNER_TO_APEX));

            // Check whether we can see the inner target
            isInnerTargetVisible = MathUtils.isInRange(-INNER_TARGET_RANGE_ANGLE, INNER_TARGET_RANGE_ANGLE, alpha);
            // Get the field oriented angle for the inner target, with latency compensation
            alpha = drivetrain.getPoseAtTime(Timer.getFPGATimestamp() - LIMELIGHT.getPipelineLatency() / 1000.0).rotation.toRadians() - alpha;
            if (false) {
                distanceToTarget = OptionalDouble.of(distanceToInnerTarget);
                angleToTarget = OptionalDouble.of(alpha);
            } else {
                distanceToTarget = OptionalDouble.of(distanceToOuterTarget);
                angleToTarget = OptionalDouble.of(tx);
            }
        } else {
            distanceToTarget = OptionalDouble.empty();
            angleToTarget = OptionalDouble.empty();
            isInnerTargetVisible = false;
        }

        // Update shuffleboard
        distanceToTargetEntry.setDouble(distanceToTarget.orElse(-1.0));
        canSeeInnerTargetEntry.setBoolean(isInnerTargetVisible);
    }

    public boolean isOnTarget() {
        return MathUtils.epsilonEquals(LIMELIGHT.getTargetPosition().x, 0.0, TARGET_ALLOWABLE_ERROR);
    }
}
