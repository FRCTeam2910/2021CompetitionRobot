package org.frcteam2910.c2020;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.common.robot.UpdateManager;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer = new RobotContainer();
    private UpdateManager updateManager = new UpdateManager(
            robotContainer.getDrivetrainSubsystem(),
            robotContainer.getWheelOfFortuneSubsystem()
    );

    @Override
    public void robotInit() {
        updateManager.startLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
