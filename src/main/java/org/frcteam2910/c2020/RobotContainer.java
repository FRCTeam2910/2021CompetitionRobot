package org.frcteam2910.c2020;

import edu.wpi.first.wpilibj2.command.*;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.XboxController;

import java.io.IOException;

public class RobotContainer {
    private static final double HOOD_MANUAL_ADJUST_INTERVAL = Math.toRadians(0.5);
    private static final double FLYWHEEL_MANUAL_ADJUST_INTERVAL = 50.0;

    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    private final XboxController secondaryController = new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
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

        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
        CommandScheduler.getInstance().registerSubsystem(visionSubsystem);
        CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
        CommandScheduler.getInstance().registerSubsystem(feederSubsystem);

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        //CommandScheduler.getInstance().setDefaultCommand(feederSubsystem, new FeederIntakeWhenNotFullCommand(feederSubsystem, 0.9));

        //CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem, new ManuallyAdjustShooterCommand(shooterSubsystem));
        CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem, new DefaultShooterCommand(shooterSubsystem, 4000, Constants.HOOD_MAX_ANGLE));
        CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, new ExtendBottomIntakeCommand(intakeSubsystem));//Remove this command as it is not needed; make the field true in the intake subsystem
        CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem,new DefaultIntakeCommand(intakeSubsystem,feederSubsystem));

        //CommandScheduler.getInstance().setDefaultCommand(feederSubsystem,new RetractTopIntakeWhenNoFifthBallCommand(feederSubsystem,intakeSubsystem));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        primaryController.getBackButton().whenPressed(
                () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );
        primaryController.getStartButton().whenPressed(
                new SpinFlywheelCommand(shooterSubsystem, 0.0)
        );


        primaryController.getLeftBumperButton().whenPressed(() -> intakeSubsystem.setTopExtended(true));

        primaryController.getLeftBumperButton().whileHeld(new SimpleIntakeCommand(intakeSubsystem, feederSubsystem, primaryController,1.0,0.9));

//        primaryController.getLeftBumperButton().whileHeld(
//                new SimpleIntakeCommand(intakeSubsystem,feederSubsystem, -1.0,0.9).withTimeout(0.25)
//                        .andThen(new SimpleIntakeCommand(intakeSubsystem, feederSubsystem,1.0,0.9)));
        //primaryController.getLeftBumperButton().whenReleased(new InstantCommand(() -> intakeSubsystem.setTopExtended(feederSubsystem.isBallAtIntake())));



        primaryController.getLeftTriggerAxis().getButton(0.5).whileHeld(new SpinFeederCommand(feederSubsystem, intakeSubsystem ,-0.5));


        primaryController.getRightTriggerAxis().getButton(0.5).whileHeld(new FeedBallsToShooterCommand(feederSubsystem, shooterSubsystem));


        primaryController.getRightBumperButton().whileHeld(
                new TargetWithShooterCommand(shooterSubsystem, visionSubsystem, primaryController).alongWith(new VisionRotateToTargetCommand(drivetrainSubsystem, visionSubsystem, () -> getDriveForwardAxis().get(true), () -> getDriveStrafeAxis().get(true)))
        );

        primaryController.getAButton().whenPressed(
                new BasicDriveCommand(drivetrainSubsystem, new Vector2(-0.5, 0.0), 0.0, false).withTimeout(0.12)
        );
        primaryController.getAButton().whileHeld(new FeedBallsToShooterCommand(feederSubsystem, shooterSubsystem));

        primaryController.getBButton().whileHeld(new IntakeCommand(intakeSubsystem, feederSubsystem, -1, true));

        primaryController.getXButton().toggleWhenPressed(new TranslationalDriveCommand(drivetrainSubsystem,getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

        //primaryController.getAButton().whileHeld(new ManuallyAdjustShooterCommand(shooterSubsystem).alongWith(new VisionRotateToTargetCommand(drivetrainSubsystem, visionSubsystem, () -> getDriveForwardAxis().get(true), () -> getDriveStrafeAxis().get(true))));

        primaryController.getYButton().whenPressed(new HomeHoodMotorCommand(shooterSubsystem));


        // Manual hood adjustment
//        primaryController.getDPadButton(DPadButton.Direction.DOWN).whenPressed(
//                () -> shooterSubsystem.setHoodTargetAngle(shooterSubsystem.getHoodTargetAngle().orElse(Constants.SHOOTER_HOOD_MAX_ANGLE) + HOOD_MANUAL_ADJUST_INTERVAL)
//        );
//        primaryController.getDPadButton(DPadButton.Direction.UP).whenPressed(
//                () -> shooterSubsystem.setHoodTargetAngle(shooterSubsystem.getHoodTargetAngle().orElse(Constants.SHOOTER_HOOD_MAX_ANGLE) - HOOD_MANUAL_ADJUST_INTERVAL)
//        );

//        // Manual flywheel adjustment
//        primaryController.getDPadButton(DPadButton.Direction.RIGHT).whenPressed(
//                () -> shooterSubsystem.shootFlywheel(shooterSubsystem.getBottomFlywheelTargetVelocity() + FLYWHEEL_MANUAL_ADJUST_INTERVAL)
//        );
//        primaryController.getDPadButton(DPadButton.Direction.LEFT).whenPressed(
//                () -> shooterSubsystem.shootFlywheel(shooterSubsystem.getBottomFlywheelTargetVelocity() - FLYWHEEL_MANUAL_ADJUST_INTERVAL)
//        );
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


    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return  visionSubsystem;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public XboxController getSecondaryController() {
        return secondaryController;
    }
}
