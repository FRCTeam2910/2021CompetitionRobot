package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;

public class FeedBallsToShooterCommand extends CommandBase {
    private final FeederSubsystem feederSubsystem;

    public FeedBallsToShooterCommand(FeederSubsystem feederSubsystem) {
        this.feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        feederSubsystem.spinMotor(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
    }
}
