package org.frcteam2910.c2020.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frcteam2910.c2020.subsystems.DrivetrainSubsystem;
import org.frcteam2910.c2020.subsystems.VisionSubsystem;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.Limelight;

import java.util.function.DoubleSupplier;

public class VisionRotateToTargetCommand extends CommandBase {
    private static final PidConstants PID_CONSTANTS = new PidConstants(1.25, 0.0, 0.0);

    private final DrivetrainSubsystem drivetrain;
    private final VisionSubsystem visionSubsystem;

    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;

    private PidController controller = new PidController(PID_CONSTANTS);
    private double lastTime = 0.0;

    public VisionRotateToTargetCommand(DrivetrainSubsystem drivetrain, VisionSubsystem visionSubsystem,
                                       DoubleSupplier xAxis, DoubleSupplier yAxis) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        addRequirements(drivetrain);
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
        lastTime = Timer.getFPGATimestamp();
        visionSubsystem.setCamMode(Limelight.CamMode.VISION);
        controller.reset();
    }

    @Override
    public void execute() {
        double time = Timer.getFPGATimestamp();
        double dt = time - lastTime;
        lastTime = time;

        double rotationalVelocity = 0.0;
        if(visionSubsystem.hasTarget()) {
            double currentAngle = drivetrain.getPose().rotation.toRadians();
            double targetAngle = visionSubsystem.getAngleToTarget().getAsDouble();
            controller.setSetpoint(targetAngle);
            rotationalVelocity = controller.calculate(currentAngle, dt);
            rotationalVelocity += Math.copySign(
                    DrivetrainSubsystem.FEEDFORWARD_CONSTANTS.getStaticConstant() / RobotController.getBatteryVoltage(),
                    rotationalVelocity
            );
        }
        drivetrain.drive(new Vector2(xAxis.getAsDouble(), yAxis.getAsDouble()), rotationalVelocity, true);
    }
}
