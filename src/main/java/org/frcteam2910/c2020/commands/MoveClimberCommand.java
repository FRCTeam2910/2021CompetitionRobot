package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.ClimberSubsystem;

public class MoveClimberCommand extends CommandBase {
    private static final double UNLOCK_WAIT_TIME = 0.25;

    private final ClimberSubsystem climber;
    private final double output;

    private final Timer timer = new Timer();

    public MoveClimberCommand(ClimberSubsystem climber, double output) {
        this.climber = climber;
        this.output = output;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.unlock();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(UNLOCK_WAIT_TIME)) {
            climber.setMotorOutput(output);
        } else {
            climber.setMotorOutput(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.setMotorOutput(0.0);
        climber.lock();
    }
}
