package org.frcteam2910.c2020;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.c2020.commands.CharacterizeFlywheelCommand;
import org.frcteam2910.c2020.commands.SpinFlywheelCommand;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.UpdateManager;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer = new RobotContainer();
    private UpdateManager updateManager = new UpdateManager(
            robotContainer.getDrivetrainSubsystem(),
            robotContainer.getFeederSubsystem(),
//            robotContainer.getWheelOfFortuneSubsystem(),
//            robotContainer.getClimberSubsystem(),
            robotContainer.getIntakeSubsystem(),
            robotContainer.getShooterSubsystem()
    );

    @Override
    public void robotInit() {
        updateManager.startLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
        robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);

//        robotContainer.getAutonomousCommand().schedule();
        new CharacterizeFlywheelCommand(robotContainer.getShooterSubsystem()).schedule();
    }

    @Override
    public void testInit() {
        robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
        robotContainer.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO);

        new SpinFlywheelCommand(robotContainer.getShooterSubsystem(), 0.0).schedule();
    }

    @Override
    public void testPeriodic() {
        NetworkTableInstance.getDefault().flush();
    }
}
