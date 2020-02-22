package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2020.Robot;
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
        autonomousModeChooser.setDefaultOption("8 Ball Auto", AutonomousMode.EIGHT_BALL);
        autonomousModeChooser.addOption("10 Ball Auto", AutonomousMode.TEN_BALL_AUTO);
        autonomousModeChooser.addOption("Circuit 10 Ball Auto", AutonomousMode.CIRCUIT_TEN_BALL_AUTO);
        autoTab.add("Mode", autonomousModeChooser);
    }

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    private SequentialCommandGroup get10BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenBallAutoPartOne());
        followAndIntake(command, container, trajectories.getTenBallAutoPartOne());
        shootAtTarget(command, container);
        //command.addCommands(new FollowTrajectoryCommand(drivetrainSubsystem, trajectories.getTenBallAutoPartTwo()));
        //command.addCommands(new TargetWithShooterCommand(shooterSubsystem, visionSubsystem, xboxController));

        return command;
    }

    private Command get8BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        //reset robot pose
        resetRobotPose(command, container, trajectories.getEightBallAutoPartOne());
        //follow first trajectory and shoot
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectories.getEightBallAutoPartOne()));
        shootAtTarget(command, container);
        //follow second trajectory and shoot
        followAndIntake(command, container, trajectories.getEightBallAutoPartTwo());

        followAndIntake(command, container, trajectories.getEightBallAutoPartThree());
        shootAtTarget(command, container);

        return command;
    }

    public Command getCircuit10BallAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        // Reset the robot pose
        resetRobotPose(command, container, trajectories.getCircuitTenBallAutoPartOne());
        // Pickup the first balls and shoot
        followAndIntake(command, container, trajectories.getCircuitTenBallAutoPartOne());
        followAndIntake(command, container, trajectories.getCircuitTenBallAutoPartTwo());
        shootAtTarget(command, container);

        // Grab from trench
        followAndIntake(command, container, trajectories.getEightBallAutoPartTwo());
        followAndIntake(command, container, trajectories.getEightBallAutoPartThree());
        shootAtTarget(command, container);

        return command;
    }

    public Command getCommand(RobotContainer container) {
        if (autonomousModeChooser.getSelected() == AutonomousMode.TEN_BALL_AUTO) {
            return get10BallAutoCommand(container);
        } else if(autonomousModeChooser.getSelected() == AutonomousMode.EIGHT_BALL) {
            return get8BallAutoCommand(container);
        } else if(autonomousModeChooser.getSelected() == AutonomousMode.CIRCUIT_TEN_BALL_AUTO) {
            return getCircuit10BallAutoCommand(container);
        }

        return get10BallAutoCommand(container);
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container) {
        command.addCommands(
                new TargetWithShooterCommand(container.getShooterSubsystem(), container.getVisionSubsystem(), container.getPrimaryController())
                        .alongWith(new VisionRotateToTargetCommand(container.getDrivetrainSubsystem(), container.getVisionSubsystem(), () -> 0.0, () -> 0.0))
                        .alongWith(
                                new WaitCommand(0.1).andThen(new AutonomousFeedCommand(container.getShooterSubsystem(), container.getFeederSubsystem(), container.getVisionSubsystem())))
                        .withTimeout(2.5));
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getIntakeSubsystem().setExtended(true)));
        command.addCommands(
                new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                        .deadlineWith(
                                new WaitCommand(0.25)
                                        .andThen(
                                                new IntakeCommand(container.getIntakeSubsystem(), container.getFeederSubsystem(), 1.0)
                                                        .alongWith(
                                                                new FeederIntakeWhenNotFullCommand(container.getFeederSubsystem(), 0.5)
                                                        ))));
        command.addCommands(new InstantCommand(() -> container.getIntakeSubsystem().setExtended(false)));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));
    }

    private enum AutonomousMode {
        EIGHT_BALL,
        TEN_BALL_AUTO,
        CIRCUIT_TEN_BALL_AUTO
    }
}
