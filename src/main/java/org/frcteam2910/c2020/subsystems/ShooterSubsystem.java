package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.robot.UpdateManager;

import java.util.OptionalDouble;


public class ShooterSubsystem implements Subsystem, UpdateManager.Updatable {
    private static final double HOOD_SENSOR_COEFFICIENT = ((2.0 * Math.PI) * Constants.SHOOTER_HOOD_GEAR_RATIO / 5.0);

    private static final double FLYWHEEL_POSITION_SENSOR_COEFFICIENT = 1.0 / 2048.0 * Constants.FLYWHEEL_GEAR_RATIO;
    private static final double FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT = FLYWHEEL_POSITION_SENSOR_COEFFICIENT * (1000.0 / 100.0) * (60.0 / 1.0);//in terms of talonfx counts

    private static final double FLYWHEEL_P = 0.5;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;

    /**
     * Flywheel regression constants
     * <p>
     * Found by doing an exponential regression using the following points:
     * (0.25, 1480)
     * (0.3, 1780)
     * (0.5, 3000)
     * (0.6, 3600)
     * (0.7, 4235)
     * (0.8, 4835)
     */
    private static final double FLYWHEEL_FF_CONSTANT = 0.00133;
    private static final double FLYWHEEL_STATIC_FRICTION_CONSTANT = 0.469;

    private static final double FLYWHEEL_CURRENT_LIMIT = 10.0;
    private static final int HOOD_CURRENT_LIMIT = 5;

    private static final double FLYWHEEL_ALLOWABLE_ERROR = 200.0;

    private final TalonFX topRightFlyWheelMotor = new TalonFX(Constants.TOP_RIGHT_SHOOTER_MOTOR_PORT);
    private final TalonFX bottomRightFlyWheelMotor = new TalonFX(Constants.BOTTOM_RIGHT_SHOOTER_MOTOR_PORT);
    private final TalonFX bottomLeftFlyWheelMotor = new TalonFX(Constants.BOTTOM_LEFT_SHOOTER_MOTOR_PORT);

    private final TalonFX hoodAngleMotor = new TalonFX(Constants.HOOD_MOTOR_PORT);
    private final AnalogInput hoodAngleEncoder = new AnalogInput(Constants.HOOD_ANGLE_ENCODER);

    private final PidController hoodController = new PidController(new PidConstants(2.0, 0.0, 0.0));

    private final NetworkTableEntry hoodAngleEntry;

    private final NetworkTableEntry hoodAngleVoltage;

    private HoodControlMode hoodControlMode = HoodControlMode.DISABLED;
    private OptionalDouble hoodTargetPosition = OptionalDouble.empty();

    public ShooterSubsystem() {
        topRightFlyWheelMotor.configFactoryDefault();
        bottomRightFlyWheelMotor.configFactoryDefault();
        bottomLeftFlyWheelMotor.configFactoryDefault();

        bottomLeftFlyWheelMotor.setInverted(true);

        bottomRightFlyWheelMotor.follow(topRightFlyWheelMotor);
        bottomLeftFlyWheelMotor.follow(topRightFlyWheelMotor);

        bottomRightFlyWheelMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        bottomRightFlyWheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
        bottomLeftFlyWheelMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        bottomLeftFlyWheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        TalonFXConfiguration flyWheelConfiguration = new TalonFXConfiguration();
        flyWheelConfiguration.slot0.kP = FLYWHEEL_P;
        flyWheelConfiguration.slot0.kI = FLYWHEEL_I;
        flyWheelConfiguration.slot0.kD = FLYWHEEL_D;
        flyWheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        flyWheelConfiguration.supplyCurrLimit.currentLimit = FLYWHEEL_CURRENT_LIMIT;
        flyWheelConfiguration.supplyCurrLimit.enable = true;
        flyWheelConfiguration.voltageCompSaturation = 11.5;

        topRightFlyWheelMotor.configAllSettings(flyWheelConfiguration);
        bottomRightFlyWheelMotor.configAllSettings(flyWheelConfiguration);
        bottomLeftFlyWheelMotor.configAllSettings(flyWheelConfiguration);

        topRightFlyWheelMotor.enableVoltageCompensation(false);
        bottomRightFlyWheelMotor.enableVoltageCompensation(false);
        bottomLeftFlyWheelMotor.enableVoltageCompensation(false);



        TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();
        hoodConfiguration.supplyCurrLimit.currentLimit = HOOD_CURRENT_LIMIT;
        hoodConfiguration.supplyCurrLimit.enable = true;

        hoodController.setInputRange(Constants.SHOOTER_HOOD_MIN_ANGLE,Constants.SHOOTER_HOOD_MAX_ANGLE);

        hoodAngleMotor.configAllSettings(hoodConfiguration);
        hoodAngleMotor.setNeutralMode(NeutralMode.Brake);
        hoodAngleMotor.setInverted(true);
        hoodAngleMotor.setSensorPhase(false);

        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        hoodAngleVoltage = tab.add("Hood Encoder Voltage",0.0)
                .withPosition(5,0)
                .withSize(1,1)
                .getEntry();
        hoodAngleEntry = tab.add("hood angle", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        tab.addNumber("Hood Target Angle", () -> Math.toDegrees(getHoodTargetAngle().orElse(Double.NaN)))
                .withPosition(0, 1)
                .withSize(1, 1);
        tab.addNumber("Hood Raw Encoder", hoodAngleMotor::getSelectedSensorPosition)
                .withPosition(0, 2)
                .withSize(1, 1);
        tab.addBoolean("Is Flywheel at Target", this::isFlywheelAtTargetVelocity)
                .withPosition(4, 1)
                .withSize(1, 1);
        tab.addNumber("Flywheel Target", this::getFlywheelTargetVelocity)
                .withPosition(4, 0)
                .withSize(1, 1);
        tab.addNumber("Flywheel Speed",this::getFlywheelVelocity)
                .withPosition(4,2)
                .withSize(1,1);

    }

    public void setFlywheelCurrentLimitEnabled(boolean enabled) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = FLYWHEEL_CURRENT_LIMIT;
        config.enable = enabled;
        topRightFlyWheelMotor.configSupplyCurrentLimit(config, 0);
    }

    public double getHoodAngle() {
        return Constants.SHOOTER_HOOD_MAX_ANGLE - (hoodAngleEncoder.getVoltage() * HOOD_SENSOR_COEFFICIENT + Constants.SHOOTER_HOOD_OFFSET);
    }

    public OptionalDouble getHoodTargetAngle() {
        return hoodTargetPosition;
    }

    public void setHoodTargetAngle(double angle) {
        hoodControlMode = HoodControlMode.POSITION;
        hoodTargetPosition = OptionalDouble.of(angle);
    }

    public void shootFlywheel(double speed) {
        double feedforward = (FLYWHEEL_FF_CONSTANT * speed + FLYWHEEL_STATIC_FRICTION_CONSTANT) / RobotController.getBatteryVoltage();

        topRightFlyWheelMotor.set(ControlMode.Velocity, -speed / FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT, DemandType.ArbitraryFeedForward, -feedforward);
    }

    public void setFlywheelOutput(double percentage) {
        topRightFlyWheelMotor.set(ControlMode.PercentOutput,-percentage);
    }

    public void stopFlywheel() {
        topRightFlyWheelMotor.set(ControlMode.Disabled,0);
    }

    @Override
    public void update(double time, double dt) {

    }

    @Override
    public void periodic() {
        switch (hoodControlMode) {
            case DISABLED:
                hoodAngleMotor.set(TalonFXControlMode.Disabled, 0.0);
                break;
            case POSITION:
                if (getHoodTargetAngle().isEmpty()) {
                    hoodAngleMotor.set(TalonFXControlMode.Disabled, 0.0);
                } else {
                    double targetAngle = getHoodTargetAngle().getAsDouble();
                    targetAngle = MathUtils.clamp(targetAngle, Constants.SHOOTER_HOOD_MIN_ANGLE, Constants.SHOOTER_HOOD_MAX_ANGLE);

                    hoodController.setSetpoint(targetAngle);
                    hoodAngleMotor.set(TalonFXControlMode.PercentOutput, hoodController.calculate(getHoodAngle(), 0.02));
                }
                break;
        }

        hoodAngleEntry.setDouble(Math.toDegrees(getHoodAngle()));
        hoodAngleVoltage.setDouble(hoodAngleEncoder.getVoltage());
    }

    public double getFlywheelPosition() {
        return -topRightFlyWheelMotor.getSensorCollection().getIntegratedSensorPosition() * FLYWHEEL_POSITION_SENSOR_COEFFICIENT;
    }

    public double getFlywheelVelocity() {
        return -topRightFlyWheelMotor.getSensorCollection().getIntegratedSensorVelocity() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public double getFlywheelTargetVelocity() {
        return -topRightFlyWheelMotor.getClosedLoopTarget() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public void resetFlywheelPosition() {
        topRightFlyWheelMotor.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    }

    public boolean isFlywheelAtTargetVelocity() {
        return MathUtils.epsilonEquals(
                getFlywheelVelocity(),
                getFlywheelTargetVelocity(),
                FLYWHEEL_ALLOWABLE_ERROR
        );
    }

    public void disableHood() {
        hoodControlMode = HoodControlMode.DISABLED;
        hoodTargetPosition = OptionalDouble.empty();
    }

    public enum HoodControlMode {
        DISABLED,
        POSITION
    }
}
