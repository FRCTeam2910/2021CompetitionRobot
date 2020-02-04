package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.input.XboxController;

public class TargetWithShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final XboxController primaryController;

    //TODO: Make this use vision
    private static final double TARGET_ANGLE = Math.toRadians(45);
    private static final double TARGET_SPEED = 2500;

    private static final double MAXIMUM_ALLOWABLE_ANGLE_RANGE = Math.toRadians(3);
    private static final double MAXIMUM_ALLOWABLE_VELOCITY_RANGE = 50;

    public TargetWithShooterCommand(ShooterSubsystem shooterSubsystem, XboxController primaryController) {
        this.shooterSubsystem = shooterSubsystem;
        this.primaryController = primaryController;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.shootFlywheel(TARGET_SPEED);
        shooterSubsystem.setHoodTargetAngle(TARGET_ANGLE);
    }

    @Override
    public void execute() {
        if(MathUtils.epsilonEquals(shooterSubsystem.getFlywheelVelocity(),  TARGET_SPEED, MAXIMUM_ALLOWABLE_VELOCITY_RANGE) && MathUtils.epsilonEquals(shooterSubsystem.getHoodAngle(), TARGET_ANGLE, MAXIMUM_ALLOWABLE_ANGLE_RANGE)) {
            primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        } else {
            primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelOutput(0.0);
        primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }
}
