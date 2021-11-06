package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.common.math.MathUtils;

public class SpinFlywheelCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final double targetVelocity;

    public SpinFlywheelCommand(ShooterSubsystem shooter, double targetVelocity) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (shooter.getTopFlywheelVelocity() < targetVelocity * 0.5) {
            shooter.setFlywheelOutput(1.0);
        } else {
            shooter.shootFlywheel(targetVelocity);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }
}
