package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;

public class SimpleIntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final double intakeSpeed;


    public SimpleIntakeCommand(IntakeSubsystem intake, double intakeSpeed) {
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setTopExtended(true);
        intake.setMotorOutput(intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTopExtended(false);
        intake.setMotorOutput(0.0);
    }
}
