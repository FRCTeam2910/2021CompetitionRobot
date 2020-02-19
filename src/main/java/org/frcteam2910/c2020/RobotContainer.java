package org.frcteam2910.c2020;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.frcteam2910.c2020.commands.DriveCommand;
import org.frcteam2910.c2020.commands.FollowTrajectoryCommand;
import org.frcteam2910.c2020.commands.IntakeCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;

import java.io.IOException;

public class RobotContainer {
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    private final XboxController secondaryController = new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
//    private final WheelOfFortuneSubsystem wheelOfFortuneSubsystem = new WheelOfFortuneSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    public RobotContainer() {
        try {
            autonomousTrajectories  = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
        } catch (IOException e) {
            e.printStackTrace();
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        CommandScheduler.getInstance().setDefaultCommand(feederSubsystem, new FeederIntakeWhenNotFullCommand(feederSubsystem, 0.3));
//        CommandScheduler.getInstance().registerSubsystem(wheelOfFortuneSubsystem);
        CommandScheduler.getInstance().registerSubsystem(climberSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
        CommandScheduler.getInstance().registerSubsystem(visionSubsystem);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        primaryController.getBackButton().whenPressed(
                () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );
        primaryController.getLeftBumperButton().whenPressed(() -> intakeSubsystem.setExtended(true));
        primaryController.getLeftBumperButton().whileHeld(new WaitCommand(0.25).andThen(new IntakeCommand(intakeSubsystem, feederSubsystem, 0.5)));
        primaryController.getLeftBumperButton().whenReleased(() -> intakeSubsystem.setExtended(false));


        primaryController.getRightTriggerAxis().getButton(0.5).whileHeld(new FeedBallsToShooterCommand(feederSubsystem));
        primaryController.getRightBumperButton().whileHeld(
                new TargetWithShooterCommand(shooterSubsystem, visionSubsystem, primaryController).alongWith(new VisionRotateToTargetCommand(drivetrainSubsystem, visionSubsystem, () -> getDriveForwardAxis().get(true), () -> getDriveStrafeAxis().get(true)))
        );

        secondaryController.getXButton().whenPressed(new DeployClimberCommand(climberSubsystem));
        secondaryController.getYButton().whenPressed(new ConditionalCommand(
                new RetractClimberCommand(climberSubsystem),
                new ExtendClimberCommand(climberSubsystem),
                climberSubsystem::isExtended
        ));
//        secondaryController.getBButton().whenPressed(wheelOfFortuneSubsystem::extendSolenoid);
//        secondaryController.getBButton().whenReleased(wheelOfFortuneSubsystem::retractSolenoid);
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrainSubsystem;
    }

    public FeederSubsystem getFeederSubsystem() {
        return feederSubsystem;
    }

    public IntakeSubsystem getIntakeSubsystem(){
        return intakeSubsystem;
    }

//    public WheelOfFortuneSubsystem getWheelOfFortuneSubsystem() {
//        return wheelOfFortuneSubsystem;
//    }

    public ClimberSubsystem getClimberSubsystem() {
        return climberSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() { return  visionSubsystem; }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public XboxController getSecondaryController() {
        return secondaryController;
    }
}
