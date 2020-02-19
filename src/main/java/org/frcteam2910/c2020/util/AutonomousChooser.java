package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.IntakeCommand;
import org.frcteam2910.c2020.commands.TargetWithShooterCommand;
import org.frcteam2910.c2020.commands.VisionRotateToTargetCommand;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.ShooterSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;
import org.frcteam2910.common.math.RigidTransform2;
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

    private SequentialCommandGroup get10BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectories.getTenBallAutoPartOne().calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));
        command.addCommands(new InstantCommand(() -> container.getIntakeSubsystem().setExtended(true)));
        command.addCommands(
                new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.getTenBallAutoPartOne())
                        .alongWith(
                                new WaitCommand(0.25)
                                        .andThen(
                                                new IntakeCommand(container.getIntakeSubsystem(), container.getFeederSubsystem(), 0.5))));
        command.addCommands(new InstantCommand(() -> container.getIntakeSubsystem().setExtended(false)));
        command.addCommands(
                new TargetWithShooterCommand(container.getShooterSubsystem(), container.getVisionSubsystem(), container.getPrimaryController())
        .alongWith(new VisionRotateToTargetCommand(container.getDrivetrainSubsystem(), container.getVisionSubsystem(), () -> 0.0, () -> 0.0))
                        .raceWith());
        //command.addCommands(new FollowTrajectoryCommand(drivetrainSubsystem, trajectories.getTenBallAutoPartTwo()));
        //command.addCommands(new TargetWithShooterCommand(shooterSubsystem, visionSubsystem, xboxController));

        return command;
    }

    public Command getCommand(RobotContainer container) {
        if (autonomousModeChooser.getSelected() == AutonomousMode.TEN_BALL_AUTO) {
            return get10BallAutoCommand(container);
        }

        return get10BallAutoCommand(container);
    }


    private enum AutonomousMode {
        EIGHT_BALL,
        TEN_BALL_AUTO
    }
}
