package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;

public class AutoFeedCommand extends CommandBase {
    private static final double TIME_UNTIL_FEED = 0.05;
    private DrivetrainSubsystem drivetrainSubsystem;
    private FeederSubsystem feederSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private VisionSubsystem visionSubsystem;
    private Timer timer = new Timer();
    public AutoFeedCommand(DrivetrainSubsystem drivetrain, FeederSubsystem feeder, ShooterSubsystem shooter, VisionSubsystem vision) {
        this.drivetrainSubsystem = drivetrain;
        this.feederSubsystem = feeder;
        this.shooterSubsystem = shooter;
        this.visionSubsystem = vision;

    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute() {
        if (visionSubsystem.isOnTarget()&& shooterSubsystem.isBottomFlywheelAtTargetVelocity() && shooterSubsystem.isHoodAtTargetAngle()){
            timer.start();
            if(timer.hasElapsed(TIME_UNTIL_FEED)) {
                feederSubsystem.spinMotor(1.0);
            } else {
                feederSubsystem.spinMotor(0.0);
            }
        }else{
            feederSubsystem.spinMotor(0.0);
            timer.stop();
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feederSubsystem.spinMotor(0.0);
    }
}
