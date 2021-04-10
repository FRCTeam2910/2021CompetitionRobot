package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
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

        autonomousModeChooser.addOption("Optimized Barrel Racing",AutonomousMode.OPTIMIZED_BARREL_RACING);
        autonomousModeChooser.addOption("Optimized Slalom",AutonomousMode.OPTIMIZED_SLALOM);
        autonomousModeChooser.addOption("Bounce Path MK2",AutonomousMode.BOUNCE_PATH);
        autonomousModeChooser.addOption("Galactic Search",AutonomousMode.GALACTIC_SEARCH);
        autoTab.add("Mode", autonomousModeChooser)
                .withSize(3, 1);
    }

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }


    public Command getOptimizedBarrelRacing(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Reset robot pos
        resetRobotPoseAndGyro(command,container,trajectories.getOptimizedBarrelRacing());

        simpleFollow(command,container,trajectories.getOptimizedBarrelRacing());

        return command;
    }

    private Command getOptimizedSlalom(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //Reset robot pos
        resetRobotPoseAndGyro(command,container,trajectories.getOptimizedSlalom());

        simpleFollow(command,container,trajectories.getOptimizedSlalom());

        return command;
    }

    private Command getBouncePathMk2(RobotContainer container){
        SequentialCommandGroup command = new SequentialCommandGroup();

        //reset robot pos
        resetRobotPoseAndGyro(command,container,trajectories.getBouncePathMk2());

        simpleFollow(command,container,trajectories.getBouncePathMk2());

        return command;
    }

    public Command getGalacticPath(RobotContainer container){
        SequentialCommandGroup sequentialCommandGroup = new SequentialCommandGroup();
        ParallelCommandGroup parallelCommand = new ParallelCommandGroup();

        double distFromARedAngle = container.getDrivetrainSubsystem().getPose().rotation.rotateBy(Rotation2.fromDegrees(180).inverse()).toRadians();
        double distFromABlueAngle = container.getDrivetrainSubsystem().getPose().rotation.rotateBy(Rotation2.fromDegrees(90).inverse()).toRadians();
        double distFromBRedAngle = container.getDrivetrainSubsystem().getPose().rotation.rotateBy(Rotation2.fromDegrees(45).inverse()).toRadians();
        double distFromBBlueAngle = container.getDrivetrainSubsystem().getPose().rotation.rotateBy(Rotation2.fromDegrees(-45).inverse()).toRadians();

        if(distFromARedAngle > Math.PI){
            distFromARedAngle = (2 * Math.PI) - distFromARedAngle;
        }
        if(distFromABlueAngle > Math.PI){
            distFromABlueAngle = (2 * Math.PI) - distFromABlueAngle;
        }
        if(distFromBRedAngle > Math.PI){
            distFromBRedAngle = (2 * Math.PI) - distFromBRedAngle;
        }
        if(distFromBBlueAngle > Math.PI){
            distFromBBlueAngle = (2 * Math.PI) - distFromBBlueAngle;
        }

        double smallestDist = Math.min(distFromARedAngle,Math.min(distFromABlueAngle,Math.min(distFromBBlueAngle,distFromBRedAngle)));

        Trajectory trajectory = null;

        if(distFromARedAngle == smallestDist){
            trajectory = trajectories.getPathARed();

        }
        else if(distFromABlueAngle == smallestDist){
            trajectory = trajectories.getPathABlue();

        }
        else if(distFromBRedAngle == smallestDist){
            trajectory = trajectories.getPathBRed();
        }
        else if(distFromBBlueAngle == smallestDist){
            trajectory = trajectories.getPathBBlue();
        }

        if(trajectory != null){
            //Reset Robot
            resetRobotPose(sequentialCommandGroup,container,trajectory);
            simpleFollow(sequentialCommandGroup,container,trajectory);

            //Run in parallel
            deployIntake(parallelCommand,container,trajectory);

            parallelCommand.addCommands(sequentialCommandGroup);

        }
        return parallelCommand;

    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case OPTIMIZED_BARREL_RACING:
                return getOptimizedBarrelRacing(container);
            case OPTIMIZED_SLALOM:
                return getOptimizedSlalom(container);
            case BOUNCE_PATH:
                return getBouncePathMk2(container);
            case GALACTIC_SEARCH:
                return getGalacticPath(container);
        }

        return getOptimizedBarrelRacing(container);//default command
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

    private void deployIntake(ParallelCommandGroup command, RobotContainer container, Trajectory trajectory){
        command.addCommands(new SimpleIntakeCommand(container.getIntakeSubsystem(),1.0));
    }

    private void resetRobotPoseAndGyro(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory){
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), container.getDrivetrainSubsystem().getPose().rotation))));
    }

    private void simpleFollow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory){
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(),trajectory));
    }

    private enum AutonomousMode {
        OPTIMIZED_BARREL_RACING,
        OPTIMIZED_SLALOM,
        BOUNCE_PATH,
        GALACTIC_SEARCH
    }
}