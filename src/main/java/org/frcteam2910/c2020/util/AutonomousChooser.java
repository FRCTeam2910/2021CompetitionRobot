package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.TargetWithShooterCommand;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.XboxController;
import org.frcteam2910.common.util.Side;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("8 Ball Auto", AutonomousMode.EIGHT_BALL);
        autonomousModeChooser.addOption("10 Ball Auto", AutonomousMode.TEN_BALL_AUTO);
        autoTab.add("Mode", autonomousModeChooser);
    }

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    private SequentialCommandGroup get10BallAutoCommand(DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem, XboxController xboxController) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new FollowTrajectoryCommand(drivetrainSubsystem, trajectories.getTenBallAutoPartOne()));
        command.addCommands(new TargetWithShooterCommand(shooterSubsystem, xboxController));
        command.addCommands(new FollowTrajectoryCommand(drivetrainSubsystem, trajectories.getTenBallAutoPartTwo()));
        command.addCommands(new TargetWithShooterCommand(shooterSubsystem, xboxController));

        return command;
    }

    public Command getCommand(DrivetrainSubsystem drivetrainSubsystem, ShooterSubsystem shooterSubsystem, XboxController xboxController) {
        if(autonomousModeChooser.getSelected() == AutonomousMode.TEN_BALL_AUTO) {
            return get10BallAutoCommand(drivetrainSubsystem, shooterSubsystem, xboxController);
        }

        return get10BallAutoCommand(drivetrainSubsystem, shooterSubsystem, xboxController);
    }


    private enum AutonomousMode {
        EIGHT_BALL,
        TEN_BALL_AUTO
    }
}
