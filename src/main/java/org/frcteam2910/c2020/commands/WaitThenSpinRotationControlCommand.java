package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem;

public class WaitThenSpinRotationControlCommand extends SequentialCommandGroup {
    public WaitThenSpinRotationControlCommand(WheelOfFortuneSubsystem spinner) {
        addCommands(
                new WaitUntilColorSensorAboveWoFCommand(spinner),
                new SpinRotationControlCommand(spinner)
        );
    }
}
