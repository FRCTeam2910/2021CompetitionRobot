package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem;
import org.frcteam2910.c2020.util.DetectedColor;

import static org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem.SPINNER_REVOLUTIONS_PER_WHEEL_SECTION;

public class SpinColorControlCommand extends CommandBase {
    private WheelOfFortuneSubsystem spinner;
    private DetectedColor desiredColor = null;

    private boolean motorStart = false;

    public SpinColorControlCommand(WheelOfFortuneSubsystem wheelOfFortuneSpinner) {
        spinner = wheelOfFortuneSpinner;
        addRequirements(spinner);
    }

    @Override
    public void initialize() {
        String gameMessage = DriverStation.getInstance().getGameSpecificMessage();
        if (gameMessage.length() > 0) {
            desiredColor = DetectedColor.convertGameMessageToColor(gameMessage);
        }
    }

    @Override
    public void execute() {
        if (!motorStart && desiredColor != null && spinner.isCloseToColorSensor()) {
            motorStart = true;
            DetectedColor currentRobotColor = spinner.getDetectedColor();
            DetectedColor fieldSensorColor = currentRobotColor.getColorOnFieldSensor();

            int numSections = fieldSensorColor.findNumSectionsAwayFromColor(desiredColor);
            spinner.resetEncoderPosition();
            spinner.spin(numSections * SPINNER_REVOLUTIONS_PER_WHEEL_SECTION);
        }
    }

    @Override
    public boolean isFinished() {
        return spinner.getDetectedColor().getColorOnFieldSensor() == desiredColor;
    }

    @Override
    public void end(boolean interrupted) {
        spinner.stopMotor();
    }
}
