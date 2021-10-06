package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;

public class AutoFeedCommand extends CommandBase {
    private static final double TIME_UNTIL_FEED = 0.05;
    private DrivetrainSubsystem drivetrainSubsystem;
    private FeederSubsystem feederSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private VisionSubsystem visionSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private Timer timer = new Timer();
    public AutoFeedCommand(DrivetrainSubsystem drivetrain, FeederSubsystem feeder, ShooterSubsystem shooter, VisionSubsystem vision, IntakeSubsystem intake) {
        this.drivetrainSubsystem = drivetrain;
        this.feederSubsystem = feeder;
        this.shooterSubsystem = shooter;
        this.visionSubsystem = vision;
        this.intakeSubsystem = intake;
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute() {
        if (visionSubsystem.isOnTarget()&& shooterSubsystem.isFlywheelAtTargetVelocity() && shooterSubsystem.isHoodAtTargetAngle()){
            timer.start();
            if(timer.hasElapsed(TIME_UNTIL_FEED)) {
                feederSubsystem.spinMotor(1.0);
                if(feederSubsystem.isFifthBallAtIntake()) {
                    intakeSubsystem.setMotorOutput(0.5);
                } else {
                    intakeSubsystem.setMotorOutput(0.0);
                }
            } else {
                feederSubsystem.spinMotor(0.0);
                intakeSubsystem.setMotorOutput(0.0);
            }
        }else{
            feederSubsystem.spinMotor(0.0);
            intakeSubsystem.setMotorOutput(0.0);
            timer.stop();
            timer.reset();
        }

    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
        intakeSubsystem.setMotorOutput(0.0);
    }
}
