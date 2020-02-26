package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;

public class TargetWithShooterCommand extends CommandBase {
    private static final InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();

    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final XboxController primaryController;

    private static final double MAXIMUM_ALLOWABLE_ANGLE_RANGE = Math.toRadians(3);
    private static final double MAXIMUM_ALLOWABLE_VELOCITY_RANGE = 50;

    static {
        SHOOTER_TUNING.put(new InterpolatingDouble(4.0 * 12.0), new Vector2(Math.toRadians(64.5), 3000));
        SHOOTER_TUNING.put(new InterpolatingDouble(5.0 * 12.0), new Vector2(Math.toRadians(64.5), 2700));
        SHOOTER_TUNING.put(new InterpolatingDouble(6.0 * 12.0), new Vector2(Math.toRadians(63.8), 2650));
        SHOOTER_TUNING.put(new InterpolatingDouble(7.0 * 12.0), new Vector2(Math.toRadians(59.8), 2750));
        SHOOTER_TUNING.put(new InterpolatingDouble(8.0 * 12.0), new Vector2(Math.toRadians(56.5), 2850));
        SHOOTER_TUNING.put(new InterpolatingDouble(9.0 * 12.0), new Vector2(Math.toRadians(53.8), 3000));
        SHOOTER_TUNING.put(new InterpolatingDouble(10.0 * 12.0), new Vector2(Math.toRadians(50.5), 3100));
        SHOOTER_TUNING.put(new InterpolatingDouble(11.0 * 12.0), new Vector2(Math.toRadians(47.0), 3250));
        SHOOTER_TUNING.put(new InterpolatingDouble(12.0 * 12.0), new Vector2(Math.toRadians(45.3), 3400));
        SHOOTER_TUNING.put(new InterpolatingDouble(13.0 * 12.0), new Vector2(Math.toRadians(41.5), 3600));
        SHOOTER_TUNING.put(new InterpolatingDouble(14.0 * 12.0), new Vector2(Math.toRadians(40.2), 3750));
        SHOOTER_TUNING.put(new InterpolatingDouble(15.0 * 12.0), new Vector2(Math.toRadians(37.1), 3950));
        SHOOTER_TUNING.put(new InterpolatingDouble(16.0 * 12.0), new Vector2(Math.toRadians(36.2), 4100));
        SHOOTER_TUNING.put(new InterpolatingDouble(17.0 * 12.0), new Vector2(Math.toRadians(34.3), 4300));
        SHOOTER_TUNING.put(new InterpolatingDouble(18.0 * 12.0), new Vector2(Math.toRadians(32.5), 4500));
        SHOOTER_TUNING.put(new InterpolatingDouble(19.0 * 12.0), new Vector2(Math.toRadians(31.0), 4700));
        SHOOTER_TUNING.put(new InterpolatingDouble(20.0 * 12.0), new Vector2(Math.toRadians(28.7), 4950));
        SHOOTER_TUNING.put(new InterpolatingDouble(21.0 * 12.0), new Vector2(Math.toRadians(28.0), 5150));
        SHOOTER_TUNING.put(new InterpolatingDouble(22.0 * 12.0), new Vector2(Math.toRadians(26.9), 5350));
        SHOOTER_TUNING.put(new InterpolatingDouble(23.0 * 12.0), new Vector2(Math.toRadians(27.2), 5300));
        SHOOTER_TUNING.put(new InterpolatingDouble(24.0 * 12.0), new Vector2(Math.toRadians(27.1), 5350));
        SHOOTER_TUNING.put(new InterpolatingDouble(25.0 * 12.0), new Vector2(Math.toRadians(27.1), 5350));
        SHOOTER_TUNING.put(new InterpolatingDouble(26.0 * 12.0), new Vector2(Math.toRadians(27.1), 5400));
        SHOOTER_TUNING.put(new InterpolatingDouble(27.0 * 12.0), new Vector2(Math.toRadians(27.1), 5450));
        SHOOTER_TUNING.put(new InterpolatingDouble(28.0 * 12.0), new Vector2(Math.toRadians(27.1), 5500));
        SHOOTER_TUNING.put(new InterpolatingDouble(29.0 * 12.0), new Vector2(Math.toRadians(27.0), 5550));
        SHOOTER_TUNING.put(new InterpolatingDouble(30.0 * 12.0), new Vector2(Math.toRadians(27.1), 5600));
    }

    public TargetWithShooterCommand(ShooterSubsystem shooterSubsystem, VisionSubsystem visionSubsystem, XboxController primaryController) {
        this.shooterSubsystem = shooterSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.primaryController = primaryController;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFlywheelCurrentLimitEnabled(false);
    }

    @Override
    public void execute() {
        Vector2 angleAndSpeed = SHOOTER_TUNING.getInterpolated(new InterpolatingDouble(visionSubsystem.getDistanceToTarget().orElse(0.0)));

        shooterSubsystem.shootFlywheel(angleAndSpeed.y);
        shooterSubsystem.setHoodTargetAngle(angleAndSpeed.x);
        if (MathUtils.epsilonEquals(shooterSubsystem.getFlywheelVelocity(), angleAndSpeed.y, MAXIMUM_ALLOWABLE_VELOCITY_RANGE) && MathUtils.epsilonEquals(shooterSubsystem.getHoodAngle(), angleAndSpeed.x, MAXIMUM_ALLOWABLE_ANGLE_RANGE)) {
            primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        } else {
            primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelCurrentLimitEnabled(true);
        shooterSubsystem.stopFlywheel();
        primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }
}
