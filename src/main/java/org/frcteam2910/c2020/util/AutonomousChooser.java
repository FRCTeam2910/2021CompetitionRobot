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
        autonomousModeChooser.addOption("Barrel Racing",AutonomousMode.BARREL_RACING);
        autonomousModeChooser.addOption("Bounce Path",AutonomousMode.BOUNCE_PATH);
        autonomousModeChooser.addOption("Slalom Path", AutonomousMode.SLALOM);
        autoTab.add("Mode", autonomousModeChooser)
        .withSize(3, 1);
    }

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    public Command getBarrelRacingAutoNavCommand(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Reset robot pos
        resetRobotPose(command,container,trajectories.getBarrelRacingMK2Part1());
        //Follow the rest of the path
        simpleFollow(command,container,trajectories.getBarrelRacingMk2Part2());

        return command;
    }

    public Command getBouncePathCommand(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Reset robot pos
        resetRobotPose(command,container,trajectories.getBouncePathPartOne());

        //Follow the rest of the path
        simpleFollow(command,container,trajectories.getBouncePathPartTwo());
        simpleFollow(command,container,trajectories.getBouncePathPartThree());
        simpleFollow(command,container,trajectories.getBouncePathPartFour());
        //simpleFollow(command,container,trajectories.getBouncePathPartFive());

        return command;
    }

    public Command getSlalomCommand(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Reset robot pos
        resetRobotPose(command,container,trajectories.getSlalomPathPartOne());
        //Follow the rest of the path
        simpleFollow(command,container,trajectories.getSlalomPathPartTwo());

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case BARREL_RACING:
                return getBarrelRacingAutoNavCommand(container);
            case BOUNCE_PATH:
                return getBouncePathCommand(container);
            case SLALOM:
                return getSlalomCommand(container);
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

    private void simpleFollow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory){
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(),trajectory));
    }

    private enum AutonomousMode {
        BARREL_RACING,
        BOUNCE_PATH,
        SLALOM,
    }
}
