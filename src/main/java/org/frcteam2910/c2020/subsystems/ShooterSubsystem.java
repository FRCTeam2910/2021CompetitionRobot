package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.util.InterpolatingDouble;
import org.frcteam2910.common.util.InterpolatingTreeMap;


public class ShooterSubsystem implements Subsystem, UpdateManager.Updatable {
    private static final double HOOD_GEAR_REDUCTION = 14.0 / 60.0;

    private static final double HOOD_SENSOR_COEFFICIENT = ((2.0 * Math.PI) * HOOD_GEAR_REDUCTION / 2048.0);
    private static final double FLYWHEEL_POSITION_SENSOR_COEFFICIENT = 1.0 / 2048.0;
    private static final double FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT = FLYWHEEL_POSITION_SENSOR_COEFFICIENT * (1000.0 / 100.0) * (60.0 / 1.0);

    private static final double FLYWHEEL_P = 0.05;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;

    /**
     * Flywheel regression constants
     *
     * Found by doing an exponential regression using the following points:
     * (2000, 0.0001775)
     * (3000, 0.0001705)
     * (4000, 0.0001672)
     * (5000, 0.000165)
     * (6000, 0.000164)
     */
    private static final double[] FLYWHEEL_FF_REGRESSION_CONSTANTS = {
            0.0000514026,
            0.999372,
            0.00016284
    };

    private static final double HOOD_P = 0.0;
    private static final double HOOD_I = 0.0;
    private static final double HOOD_D = 0.0;

    private final TalonFX flywheelMotor1 = new TalonFX(Constants.SHOOTER_DRIVE_MOTOR_PORT_1);
    private final TalonFX flywheelMotor2 = new TalonFX(Constants.SHOOTER_DRIVE_MOTOR_PORT_2);

    private final TalonSRX angleMotor = new TalonSRX(Constants.SHOOTER_ANGLE_MOTOR_PORT);

    private final NetworkTableEntry hoodAngleEntry;
    private final NetworkTableEntry flyWheelMotor1SpeedEntry;
    private final NetworkTableEntry flywheelMotor1VoltageEntry;
    private final NetworkTableEntry flywheelMotor1CurrentEntry;
    private final NetworkTableEntry flyWheelMotor2SpeedEntry;
    private final NetworkTableEntry flywheelMotor2VoltageEntry;
    private final NetworkTableEntry flywheelMotor2CurrentEntry;

    public ShooterSubsystem() {
        flywheelMotor1.configFactoryDefault();
        flywheelMotor2.configFactoryDefault();

        TalonFXConfiguration flyWheelConfiguration = new TalonFXConfiguration();
        flyWheelConfiguration.slot0.kP = FLYWHEEL_P;
        flyWheelConfiguration.slot0.kI = FLYWHEEL_I;
        flyWheelConfiguration.slot0.kD = FLYWHEEL_D;
        flyWheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        flyWheelConfiguration.supplyCurrLimit.currentLimit = 40.0;
        flyWheelConfiguration.supplyCurrLimit.enable = true;
        flyWheelConfiguration.voltageCompSaturation = 11.5;


        flywheelMotor1.configAllSettings(flyWheelConfiguration);
        flywheelMotor2.configAllSettings(flyWheelConfiguration);

        flywheelMotor1.enableVoltageCompensation(true);
        flywheelMotor2.enableVoltageCompensation(true);

        TalonSRXConfiguration hoodConfiguration = new TalonSRXConfiguration();
        hoodConfiguration.slot0.kP = HOOD_P;
        hoodConfiguration.slot0.kI = HOOD_I;
        hoodConfiguration.slot0.kD = HOOD_D;
        hoodConfiguration.feedbackNotContinuous = false;
        hoodConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;
        hoodConfiguration.primaryPID.selectedFeedbackCoefficient = HOOD_SENSOR_COEFFICIENT;

        angleMotor.configAllSettings(hoodConfiguration);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        hoodAngleEntry = tab.add("hood angle", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        flyWheelMotor1SpeedEntry = tab.add("Wheel 1 Speed", 0.0)
                .withPosition(1, 0)
                .withSize(1, 1)
                .getEntry();
        flywheelMotor1VoltageEntry = tab.add("Wheel 1 Voltage", 0.0)
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();
        flywheelMotor1CurrentEntry = tab.add("Wheel 1 Current", 0.0)
                .withPosition(3, 0)
                .withSize(1, 1)
                .getEntry();
        flyWheelMotor2SpeedEntry = tab.add("Wheel 2 Speed", 0.0)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();
        flywheelMotor2VoltageEntry = tab.add("Wheel 2 Voltage", 0.0)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();
        flywheelMotor2CurrentEntry = tab.add("Wheel 2 Current", 0.0)
                .withPosition(3, 1)
                .withSize(1, 1)
                .getEntry();
    }

    private double calculateFlywheelFeedforward(double velocity) {
        return FLYWHEEL_FF_REGRESSION_CONSTANTS[0] * Math.pow(FLYWHEEL_FF_REGRESSION_CONSTANTS[1], velocity) + FLYWHEEL_FF_REGRESSION_CONSTANTS[2];
    }

    public double getHoodAngle() {
        return angleMotor.getSelectedSensorPosition();
    }

    public void setHoodTargetAngle(double angle) {
        angleMotor.set(ControlMode.Position, angle);
    }

    public void shootFlywheel(double speed) {
        double feedforward = calculateFlywheelFeedforward(speed) * speed;

        flywheelMotor1.set(ControlMode.Velocity, -speed / FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT, DemandType.ArbitraryFeedForward, -feedforward);
        flywheelMotor2.set(ControlMode.Velocity, speed / FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT, DemandType.ArbitraryFeedForward, feedforward);
    }

    public void setFlywheelOutput(double percentage) {
        flywheelMotor1.set(ControlMode.PercentOutput, -percentage);
        flywheelMotor2.set(ControlMode.PercentOutput, percentage);
    }

    public void stopFlywheel() {
        flywheelMotor1.set(ControlMode.Disabled, 0);
        flywheelMotor2.set(ControlMode.Disabled, 0);
    }

    @Override
    public void update(double time, double dt) {

    }

    @Override
    public void periodic() {
        hoodAngleEntry.setDouble(getHoodAngle());
        flyWheelMotor1SpeedEntry.setDouble(-flywheelMotor1.getSensorCollection().getIntegratedSensorVelocity() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT);
        flywheelMotor1VoltageEntry.setDouble(-flywheelMotor1.getMotorOutputVoltage());
        flywheelMotor1CurrentEntry.setDouble(flywheelMotor1.getSupplyCurrent());
        flyWheelMotor2SpeedEntry.setDouble(flywheelMotor2.getSensorCollection().getIntegratedSensorVelocity() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT);
        flywheelMotor2VoltageEntry.setDouble(flywheelMotor2.getMotorOutputVoltage());
        flywheelMotor2CurrentEntry.setDouble(flywheelMotor2.getSupplyCurrent());
    }

    public double getFlywheelPosition() {
        return -flywheelMotor1.getSensorCollection().getIntegratedSensorPosition() * FLYWHEEL_POSITION_SENSOR_COEFFICIENT;
    }

    public double getFlywheelVelocity() {
        return -flywheelMotor1.getSensorCollection().getIntegratedSensorVelocity() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public void resetFlywheelPosition() {
        flywheelMotor1.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    }
}
