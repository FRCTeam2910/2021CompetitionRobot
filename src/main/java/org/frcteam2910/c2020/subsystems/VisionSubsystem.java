package org.frcteam2910.c2020.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;

import java.util.OptionalDouble;

public class VisionSubsystem implements Subsystem {
    private static final double TARGET_HEIGHT = 98.25;
    private static final double LIMELIGHT_MOUTNING_ANGLE = Math.toRadians(20.0);

    private final Limelight limelight;

    private final NetworkTableEntry distanceToTargetEntry;

    private OptionalDouble distanceToTarget;

    public VisionSubsystem() {
        limelight = new Limelight();

        ShuffleboardTab tab = Shuffleboard.getTab("Vision");
        distanceToTargetEntry = tab.add("distance to target", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
    }

    public OptionalDouble getDistanceToTarget() {
        return distanceToTarget;
    }

    @Override
    public void periodic() {
        // Determine whether the Limelight has a target or not
        if (limelight.hasTarget()) {
            // If so, get the target position and calculate the distance to it
            Vector2 targetPosition = limelight.getTargetPosition();
            double theta = LIMELIGHT_MOUTNING_ANGLE + targetPosition.y;
            distanceToTarget = OptionalDouble.of(TARGET_HEIGHT /  Math.tan(theta));
            distanceToTargetEntry.setDouble(distanceToTarget.getAsDouble());
        } else {
            // Else, set distanceToTarget to an empty OptionalDouble
            distanceToTarget = OptionalDouble.empty();
            distanceToTargetEntry.setDouble(-1.0);
        }
    }
}
