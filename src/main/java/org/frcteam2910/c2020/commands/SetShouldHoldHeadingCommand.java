package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Vector2;

public class SetShouldHoldHeadingCommand extends CommandBase {


    private DrivetrainSubsystem drivetrainSubsystem;

    public SetShouldHoldHeadingCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute() {
        drivetrainSubsystem.setShouldHoldHeading();
    }
}
