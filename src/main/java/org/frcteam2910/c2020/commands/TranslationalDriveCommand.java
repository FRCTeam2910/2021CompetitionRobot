package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;

public class TranslationalDriveCommand extends CommandBase {


    private DrivetrainSubsystem drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;

    public TranslationalDriveCommand(DrivetrainSubsystem drivetrainSubsystem, Axis forward, Axis strafe, Axis rotation) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        addRequirements(drivetrainSubsystem);
    }


    @Override
    public void initialize() {
        drivetrainSubsystem.setShouldHoldHeading();
    }

    @Override
    public void execute() {

        drivetrainSubsystem.drive(new Vector2(forward.get(true), strafe.get(true)), 0, true,true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.setShouldHoldHeading();
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false,true);
    }
}
