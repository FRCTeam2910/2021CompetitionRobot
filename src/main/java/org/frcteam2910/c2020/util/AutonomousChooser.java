package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.addOption("Barrel Racing - AutoNav",AutonomousMode.BARREL_RACING_AUTO_NAV);
        autoTab.add("Mode", autonomousModeChooser)
        .withSize(3, 1);
    }

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    public Command getBarrelRacingAutoNavCommand(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Reset robot pose
        resetRobotPose(command, container, trajectories.getBarrelRacingAutoNavPartOne());
        //Set the auto path
        follow(command,container, trajectories.getBarrelRacingAutoNavPartTwo());

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case BARREL_RACING_AUTO_NAV:
                return getBarrelRacingAutoNavCommand(container);
        }

        return getBarrelRacingAutoNavCommand(container);//default command
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container) {
        shootAtTarget(command, container, 2.5);
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container, double timeToWait) {
        command.addCommands(
                new TargetWithShooterCommand(container.getShooterSubsystem(), container.getVisionSubsystem(), container.getPrimaryController())
                        .alongWith(new VisionRotateToTargetCommand(container.getDrivetrainSubsystem(), container.getVisionSubsystem(), () -> 0.0, () -> 0.0))
                        .alongWith(
                                new WaitCommand(0.1).andThen(new AutonomousFeedCommand(container.getShooterSubsystem(), container.getFeederSubsystem(), container.getVisionSubsystem())))
                        .withTimeout(timeToWait));
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(new TargetWithShooterCommand(container.getShooterSubsystem(), container.getVisionSubsystem(), container.getPrimaryController()))
                .alongWith(new PrepareBallsToShootCommand(container.getFeederSubsystem(), 1.0)));
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getIntakeSubsystem().setExtended(true)));
        command.addCommands(
                new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                        .deadlineWith(
                                new IntakeCommand(container.getIntakeSubsystem(), container.getFeederSubsystem(), -1.0).withTimeout(0.25)
                                        .andThen(
                                                new IntakeCommand(container.getIntakeSubsystem(), container.getFeederSubsystem(), 1.0)
                                                        .alongWith(
                                                                new FeederIntakeWhenNotFullCommand(container.getFeederSubsystem(), 1.0)
                                                        ))));
        command.addCommands(new InstantCommand(() -> container.getIntakeSubsystem().setExtended(false)));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));
    }

    private enum AutonomousMode {
        BARREL_RACING_AUTO_NAV
    }
}
