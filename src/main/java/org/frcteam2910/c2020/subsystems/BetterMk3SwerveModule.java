package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Talon;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Utilities;

public class BetterMk3SwerveModule extends SwerveModule {

    private static final double DRIVE_GEAR_RATIO = 6.86;
    private static final double STEERING_GEAR_RATIO = 12.8;
    private static final double WHEEL_RADIUS = 2;

    private static final PidConstants ANGLE_CONSTANTS = new PidConstants(0,0,0);

    private static final double DRIVE_COUNTS_TO_INCHES = 1 / 2048.0 / DRIVE_GEAR_RATIO * (2 * WHEEL_RADIUS * Math.PI);

    private TalonFX driveMotor;
    private TalonFX steeringMotor;
    private CANCoder angleEncoder;
    private double angleOffset;

    private PidController anglePIDController;


    public BetterMk3SwerveModule(Vector2 modulePosition,
                                 TalonFX driveMotor,
                                 TalonFX steeringMotor,
                                 CANCoder angleEncoder,
                                 double angleOffset) {

        super(modulePosition);
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.angleEncoder = angleEncoder;
        this.angleOffset = angleOffset;

        this.anglePIDController = new PidController(ANGLE_CONSTANTS);
        this.anglePIDController.setInputRange(0, 2 * Math.PI);
        this.anglePIDController.setContinuous(true);

    }

    @Override
    protected double readAngle() {
        System.out.println(angleEncoder.getAbsolutePosition());
        double unadjustedAngleRadians = Math.toRadians(angleEncoder.getAbsolutePosition()) + angleOffset;
        double clampedAngle = unadjustedAngleRadians % 2 * Math.PI;

        if (clampedAngle < 0) {
            clampedAngle += 2 * Math.PI;
        }

        return clampedAngle;
    }

    @Override
    protected double readDistance() {
        return driveMotor.getSensorCollection().getIntegratedSensorPosition() * DRIVE_COUNTS_TO_INCHES;
    }

    @Override
    protected void setTargetAngle(double angle) {
        anglePIDController.setSetpoint(angle);
        double angleOutput = anglePIDController.calculate(readAngle(),5e-3);
        steeringMotor.set(TalonFXControlMode.PercentOutput,angleOutput);
    }

    @Override
    protected void setDriveOutput(double output) {
        driveMotor.set(TalonFXControlMode.PercentOutput,output);
    }
}
