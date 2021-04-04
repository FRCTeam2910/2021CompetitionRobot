package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.CircularBuffer;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;
import org.frcteam2910.common.util.MovingAverage;

public class TargetWithShooterCommand extends CommandBase {
    private static final InterpolatingTreeMap<InterpolatingDouble, Vector2> SHOOTER_TUNING = new InterpolatingTreeMap<>();

    private static final double MAXIMUM_ALLOWABLE_ANGLE_RANGE = Math.toRadians(3);
    private static final double MAXIMUM_ALLOWABLE_VELOCITY_RANGE = 50;

    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final XboxController primaryController;

    static {
        SHOOTER_TUNING.put(new InterpolatingDouble(86.0),new Vector2(Math.toRadians(52.19),2431));
        SHOOTER_TUNING.put(new InterpolatingDouble(124.25),new Vector2(Math.toRadians(43.3),2778.6));
        SHOOTER_TUNING.put(new InterpolatingDouble(152.0),new Vector2(Math.toRadians(37.7),3076));
        SHOOTER_TUNING.put(new InterpolatingDouble(186.2),new Vector2(Math.toRadians(31.68),3474));
        SHOOTER_TUNING.put(new InterpolatingDouble(219.0),new Vector2(Math.toRadians(27.7),3821.5));
        SHOOTER_TUNING.put(new InterpolatingDouble(242.3),new Vector2(Math.toRadians(24.7),4169));
        SHOOTER_TUNING.put(new InterpolatingDouble(281.0),new Vector2(Math.toRadians(22.2),4467));
        SHOOTER_TUNING.put(new InterpolatingDouble(324.0),new Vector2(Math.toRadians(20.2),4616));
        SHOOTER_TUNING.put(new InterpolatingDouble(371.0),new Vector2(Math.toRadians(18.7),4863.8));
        SHOOTER_TUNING.put(new InterpolatingDouble(409.0),new Vector2(Math.toRadians(17.2),5211.5));
        SHOOTER_TUNING.put(new InterpolatingDouble(491.0),new Vector2(Math.toRadians(15.2),5509.5));


//        SHOOTER_TUNING.put(new InterpolatingDouble(88.39),new Vector2(Math.toRadians(52.69),2400));
//        SHOOTER_TUNING.put(new InterpolatingDouble(125.7),new Vector2(Math.toRadians(42.67),2866));
//        SHOOTER_TUNING.put(new InterpolatingDouble(154.0),new Vector2(Math.toRadians(35.65),3313));
//        SHOOTER_TUNING.put(new InterpolatingDouble(193.37),new Vector2(Math.toRadians(32.12),3610));
//        SHOOTER_TUNING.put(new InterpolatingDouble(232.6),new Vector2(Math.toRadians(27.1),4000));
//        SHOOTER_TUNING.put(new InterpolatingDouble(284.0),new Vector2(Math.toRadians(24.5),4320));
//        SHOOTER_TUNING.put(new InterpolatingDouble(331.5),new Vector2(Math.toRadians(21.41),4965));
//        SHOOTER_TUNING.put(new InterpolatingDouble(455.0),new Vector2(Math.toRadians(17.4),6060));

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
        if (MathUtils.epsilonEquals(shooterSubsystem.getTopFlywheelVelocity(), angleAndSpeed.y, MAXIMUM_ALLOWABLE_VELOCITY_RANGE) && MathUtils.epsilonEquals(shooterSubsystem.getHoodMotorAngle(), angleAndSpeed.x, MAXIMUM_ALLOWABLE_ANGLE_RANGE)) {
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
