package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;

public class HoldFifthBallCommand extends CommandBase {

    private FeederSubsystem feederSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public HoldFifthBallCommand(FeederSubsystem feederSubsystem, IntakeSubsystem intakeSubsystem){
        this.feederSubsystem = feederSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(feederSubsystem);
    }


    @Override
    public void execute() {
        if(feederSubsystem.isFifthBallAtIntake()){
            intakeSubsystem.setTopExtended(true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
