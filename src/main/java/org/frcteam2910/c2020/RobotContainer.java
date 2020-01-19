package org.frcteam2910.c2020;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frcteam2910.c2020.commands.DriveCommand;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.FeederSubsystem;
import org.frcteam2910.c2020.subsystems.WheelOfFortuneSubsystem;
import org.frcteam2910.c2020.subsystems.IntakeSubsystem;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.input.Axis;
import org.frcteam2910.common.robot.input.Controller;
import org.frcteam2910.common.robot.input.XboxController;

public class RobotContainer {
    private final Controller primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final FeederSubsystem feederSubsystem = new FeederSubsystem();
    private final WheelOfFortuneSubsystem wheelOfFortuneSubsystem = new WheelOfFortuneSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public RobotContainer() {
        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        CommandScheduler.getInstance().registerSubsystem(feederSubsystem);
        CommandScheduler.getInstance().registerSubsystem(wheelOfFortuneSubsystem);
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        primaryController.getBackButton().whenPressed(
                () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );
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

    public FeederSubsystem getFeederSubsystem(){
        return feederSubsystem;
    }



    public WheelOfFortuneSubsystem getWheelOfFortuneSubsystem(){
        return wheelOfFortuneSubsystem;
    }

}
