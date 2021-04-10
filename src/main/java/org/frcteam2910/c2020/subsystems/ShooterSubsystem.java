package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.control.MotionProfileFollower;
import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.MathUtils;
import org.frcteam2910.common.motion.MotionProfile;
import org.frcteam2910.common.motion.TrapezoidalMotionProfile;
import org.frcteam2910.common.robot.UpdateManager;
import org.opencv.core.Mat;

import java.util.OptionalDouble;


public class ShooterSubsystem implements Subsystem, UpdateManager.Updatable {

    private static final double FLYWHEEL_POSITION_SENSOR_COEFFICIENT = 1.0 / 2048.0 * Constants.FLYWHEEL_GEAR_RATIO;
    private static final double FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT = FLYWHEEL_POSITION_SENSOR_COEFFICIENT * (1000.0 / 100.0) * (60.0);//in terms of talonfx counts

    private static final double FLYWHEEL_ALLOWABLE_ERROR = 200.0;

    private static final double TOP_FLYWHEEL_P = 0.05;
    private static final double TOP_FLYWHEEL_I = 0.0;
    private static final double TOP_FLYWHEEL_D = 0.0;

    private static final double BOTTOM_FLYWHEEL_P = 0.5;
    private static final double BOTTOM_FLYWHEEL_I = 0.0;
    private static final double BOTTOM_FLYWHEEL_D = 0.0;

    private static final double TOP_FLYWHEEL_FF_CONSTANT = 0.00127;
    private static final double TOP_FLYWHEEL_STATIC_FRICTION_CONSTANT = 0.67002;

    private static final double BOTTOM_FLYWHEEL_FF_CONSTANT = 0.0012148;
    private static final double BOTTOM_FLYWHEEL_STATIC_FRICTION_CONSTANT = 0.5445;

    private static final double HOOD_ANGLE_P = 0.5;
    private static final double HOOD_ANGLE_I = 0;
    private static final double HOOD_ANGLE_D = 5;

    private static final double BOTTOM_FLYWHEEL_CURRENT_LIMIT = 10.0;
    private static final int HOOD_CURRENT_LIMIT = 15;

    private final TalonFX bottomFlywheelPrimaryMotor = new TalonFX(Constants.TOP_RIGHT_SHOOTER_MOTOR_PORT);
    private final TalonFX bottomFlywheelSecondaryMotor = new TalonFX(Constants.BOTTOM_RIGHT_SHOOTER_MOTOR_PORT);
    private final TalonFX topFlywheelMotor = new TalonFX(Constants.BOTTOM_LEFT_SHOOTER_MOTOR_PORT);

    private final TalonFX hoodAngleMotor = new TalonFX(Constants.HOOD_MOTOR_PORT);

    private final NetworkTableEntry hoodAngleEntry;

    private boolean isHoodHomed;

    private HoodControlMode hoodControlMode = HoodControlMode.DISABLED;
    private OptionalDouble hoodTargetPosition = OptionalDouble.empty();
    private double hoodPercentOutput = 0.0;

    public ShooterSubsystem() {
        isHoodHomed = false;

        topFlywheelMotor.configFactoryDefault();
        bottomFlywheelPrimaryMotor.configFactoryDefault();
        bottomFlywheelSecondaryMotor.configFactoryDefault();

        //Save CAN bandwidth
        bottomFlywheelSecondaryMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        bottomFlywheelSecondaryMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        topFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        topFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        hoodAngleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        hoodAngleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        TalonFXConfiguration topFlyWheelConfiguration = new TalonFXConfiguration();
        topFlyWheelConfiguration.slot0.kP = TOP_FLYWHEEL_P;
        topFlyWheelConfiguration.slot0.kI = TOP_FLYWHEEL_I;
        topFlyWheelConfiguration.slot0.kD = TOP_FLYWHEEL_D;
        topFlyWheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        topFlyWheelConfiguration.supplyCurrLimit.currentLimit = BOTTOM_FLYWHEEL_CURRENT_LIMIT;
        topFlyWheelConfiguration.supplyCurrLimit.enable = true;
        topFlyWheelConfiguration.voltageCompSaturation = 11.5;

        topFlywheelMotor.configAllSettings(topFlyWheelConfiguration);
        topFlywheelMotor.enableVoltageCompensation(false);



        TalonFXConfiguration bottomFlyWheelConfiguration = new TalonFXConfiguration();
        bottomFlyWheelConfiguration.slot0.kP = BOTTOM_FLYWHEEL_P;
        bottomFlyWheelConfiguration.slot0.kI = BOTTOM_FLYWHEEL_I;
        bottomFlyWheelConfiguration.slot0.kD = BOTTOM_FLYWHEEL_D;
        bottomFlyWheelConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        bottomFlyWheelConfiguration.supplyCurrLimit.currentLimit = BOTTOM_FLYWHEEL_CURRENT_LIMIT;
        bottomFlyWheelConfiguration.supplyCurrLimit.enable = true;
        bottomFlyWheelConfiguration.voltageCompSaturation = 11.5;

        bottomFlywheelPrimaryMotor.configAllSettings(bottomFlyWheelConfiguration);
        bottomFlywheelSecondaryMotor.configAllSettings(bottomFlyWheelConfiguration);

        bottomFlywheelPrimaryMotor.enableVoltageCompensation(false);
        bottomFlywheelSecondaryMotor.enableVoltageCompensation(false);

        bottomFlywheelSecondaryMotor.follow(bottomFlywheelPrimaryMotor);



        TalonFXConfiguration hoodConfiguration = new TalonFXConfiguration();
        hoodConfiguration.slot0.kP = HOOD_ANGLE_P;
        hoodConfiguration.slot0.kI = HOOD_ANGLE_I;
        hoodConfiguration.slot0.kD = HOOD_ANGLE_D;
        hoodConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        hoodConfiguration.supplyCurrLimit.currentLimit = HOOD_CURRENT_LIMIT;
        hoodConfiguration.supplyCurrLimit.enable = true;

        hoodAngleMotor.configAllSettings(hoodConfiguration);
        hoodAngleMotor.setNeutralMode(NeutralMode.Coast);
        hoodAngleMotor.setSensorPhase(false);
        hoodAngleMotor.setInverted(true);


        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
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
        tab.addBoolean("Is Top Flywheel at Target", this::isTopFlywheelAtTargetVelocity)
                .withPosition(4, 1)
                .withSize(1, 1);
        tab.addNumber("Top Flywheel Target", this::getTopFlywheelTargetVelocity)
                .withPosition(4, 0)
                .withSize(1, 1);
        tab.addNumber("Top Flywheel Speed", this::getTopFlywheelVelocity)
                .withPosition(4, 2)
                .withSize(1, 1);
        tab.addBoolean("Is Bottom Flywheel at Target", this::isBottomFlywheelAtTargetVelocity)
                .withPosition(2, 1)
                .withSize(1, 1);
        tab.addNumber("Bottom Flywheel Target", this::getBottomFlywheelTargetVelocity)
                .withPosition(2, 0)
                .withSize(1, 1);
        tab.addNumber("Bottom Flywheel Speed", this::getBottomFlywheelVelocity)
                .withPosition(2, 2)
                .withSize(1, 1);
        tab.addNumber("Angle Error", this::angleTargetError)
                .withPosition(1, 1)
                .withSize(1, 1);
        tab.addBoolean("Hood Homed", this::isHoodHomed)
                .withPosition(1, 0)
                .withSize(1, 1);

    }

    public void setFlywheelCurrentLimitEnabled(boolean enabled) {
        SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration();
        config.currentLimit = BOTTOM_FLYWHEEL_CURRENT_LIMIT;
        config.enable = enabled;
        bottomFlywheelPrimaryMotor.configSupplyCurrentLimit(config, 0);
        bottomFlywheelSecondaryMotor.configSupplyCurrentLimit(config, 0);
        topFlywheelMotor.configSupplyCurrentLimit(config, 0);
    }

    public OptionalDouble getHoodTargetAngle() {
        return hoodTargetPosition;
    }

    public void setHoodTargetAngle(double angle) {
        hoodControlMode = HoodControlMode.POSITION;
        hoodTargetPosition = OptionalDouble.of(angle);
    }

    public void shootFlywheel(double speed) {
        double topFlyWheelSpeed = speed * (4 / 2.5 / 3);

        double bottomFeedforward = (BOTTOM_FLYWHEEL_FF_CONSTANT * speed + BOTTOM_FLYWHEEL_STATIC_FRICTION_CONSTANT) / RobotController.getBatteryVoltage();
        double topFeedforward = (TOP_FLYWHEEL_FF_CONSTANT * topFlyWheelSpeed + TOP_FLYWHEEL_STATIC_FRICTION_CONSTANT) / RobotController.getBatteryVoltage();

        bottomFlywheelPrimaryMotor.set(ControlMode.Velocity, -speed / FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT, DemandType.ArbitraryFeedForward, -bottomFeedforward);
        topFlywheelMotor.set(ControlMode.Velocity, -topFlyWheelSpeed / FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT, DemandType.ArbitraryFeedForward, -topFeedforward);
    }

    public void setFlywheelOutput(double percentage) {
        bottomFlywheelPrimaryMotor.set(ControlMode.PercentOutput, -percentage);
        topFlywheelMotor.set(ControlMode.PercentOutput, percentage);
    }

    public void stopFlywheel() {
        bottomFlywheelPrimaryMotor.set(ControlMode.Disabled, 0);
        topFlywheelMotor.set(ControlMode.Disabled, 0);
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
                if(!isHoodHomed){
                    break;
                }

                if (getHoodTargetAngle().isEmpty()) {
                    hoodAngleMotor.set(TalonFXControlMode.Disabled, 0.0);
                } else {
                    double targetAngle = getHoodTargetAngle().getAsDouble();
                    targetAngle = MathUtils.clamp(targetAngle, Constants.SHOOTER_HOOD_MIN_ANGLE, Constants.SHOOTER_HOOD_MAX_ANGLE);

                    hoodAngleMotor.set(TalonFXControlMode.Position, angleToTalonUnits(targetAngle));

                }
                break;
            case PERCENT_OUTPUT:
                this.hoodAngleMotor.set(ControlMode.PercentOutput, hoodPercentOutput);
                break;
        }

        hoodAngleEntry.setDouble(Math.toDegrees(getHoodMotorAngle()));
    }

    public double getTopFlywheelPosition() {
        return -bottomFlywheelPrimaryMotor.getSensorCollection().getIntegratedSensorPosition() * FLYWHEEL_POSITION_SENSOR_COEFFICIENT;
    }

    public double getBottomFlywheelPosition() {
        return -bottomFlywheelPrimaryMotor.getSensorCollection().getIntegratedSensorPosition() * FLYWHEEL_POSITION_SENSOR_COEFFICIENT;
    }

    public double getTopFlywheelVelocity() {
        return -topFlywheelMotor.getSensorCollection().getIntegratedSensorVelocity() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public double getBottomFlywheelVelocity() {
        return -bottomFlywheelPrimaryMotor.getSensorCollection().getIntegratedSensorVelocity() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public double getTopFlywheelTargetVelocity() {
        return -topFlywheelMotor.getClosedLoopTarget() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public double getBottomFlywheelTargetVelocity() {
        return -bottomFlywheelPrimaryMotor.getClosedLoopTarget() * FLYWHEEL_VELOCITY_SENSOR_COEFFICIENT;
    }

    public void resetTopFlywheelPosition() {
        bottomFlywheelPrimaryMotor.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    }

    public void resetBottomFlywheelPosition() {
        bottomFlywheelPrimaryMotor.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    }

    public boolean isTopFlywheelAtTargetVelocity() {
        return MathUtils.epsilonEquals(
                getTopFlywheelVelocity(),
                getTopFlywheelTargetVelocity(),
                FLYWHEEL_ALLOWABLE_ERROR
        );
    }

    public boolean isBottomFlywheelAtTargetVelocity() {
        return MathUtils.epsilonEquals(
                getBottomFlywheelVelocity(),
                getBottomFlywheelTargetVelocity(),
                FLYWHEEL_ALLOWABLE_ERROR
        );
    }

    private double angleTargetError() {
        if (getHoodTargetAngle().isPresent()) {
            return Math.toDegrees(getHoodTargetAngle().getAsDouble() - getHoodMotorAngle());
        }

        return 0.0;

    }

    public void disableHood() {
        hoodControlMode = HoodControlMode.DISABLED;
        hoodTargetPosition = OptionalDouble.empty();
    }

    public double getHoodVelocity() {
        return -hoodAngleMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 / 2048 * (2 * Math.PI) * Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;
    }

    public void zeroHoodMotor() {
        this.isHoodHomed = true;

        double sensorPosition = (Constants.SHOOTER_HOOD_MAX_ANGLE + Math.toRadians(1)) * 2048 / (2 * Math.PI) * Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;
        hoodAngleMotor.setSelectedSensorPosition((int) sensorPosition);
    }

    private double talonUnitsToHoodAngle(double talonUnits) {
        double angle = -talonUnits / 2048 * (2 * Math.PI) / Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;

        return angle;
    }

    private double angleToTalonUnits(double angle){
        double talonUnits = angle * 2048 / (2 * Math.PI) * Constants.HOOD_MOTOR_TO_HOOD_GEAR_RATIO;
        return talonUnits;
    }

    public void setHoodMotorPower(double percent) {
        hoodControlMode = HoodControlMode.PERCENT_OUTPUT;
        hoodPercentOutput = percent;
    }

    public boolean isHoodHomed() {
        return isHoodHomed;
    }

    public void setHoodHomed(boolean target) {
        this.isHoodHomed = target;
    }

    public double getHoodMotorAngle() {
        return talonUnitsToHoodAngle(hoodAngleMotor.getSensorCollection().getIntegratedSensorPosition());
    }

    public enum HoodControlMode {
        DISABLED,
        POSITION,
        PERCENT_OUTPUT
    }
}
