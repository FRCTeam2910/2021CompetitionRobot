package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.util.HolonomicDriveSignal;


public class DrivetrainSubsystem implements Subsystem, UpdateManager.Updatable {
    public static final double TRACKWIDTH = 1.0;
    public static final double WHEELBASE = 1.0;

    private final SwerveModule frontLeftModule =
            new Mk2SwerveModuleBuilder(new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleMotor(
                    new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR),
                    Mk2SwerveModuleBuilder.MotorType.CIM)
            .angleEncoder(
                    new AnalogInput(Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT),
                    Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET)
            .build();

    private final SwerveModule frontRightModule =
            new Mk2SwerveModuleBuilder(new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleMotor(
                    new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new WPI_TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR),
                    Mk2SwerveModuleBuilder.MotorType.CIM)
            .angleEncoder(
                    new AnalogInput(Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT),
                    Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET)
            .build();

    private final SwerveModule backLeftModule =
            new Mk2SwerveModuleBuilder(new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleMotor(
                    new CANSparkMax(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new WPI_TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR),
                    Mk2SwerveModuleBuilder.MotorType.CIM)
            .angleEncoder(
                    new AnalogInput(Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT),
                    Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET)
            .build();

    private final SwerveModule backRightModule =
            new Mk2SwerveModuleBuilder(new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleMotor(
                    new CANSparkMax(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    Mk2SwerveModuleBuilder.MotorType.NEO)
            .driveMotor(
                    new WPI_TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR),
                    Mk2SwerveModuleBuilder.MotorType.CIM)
            .angleEncoder(
                    new AnalogInput(Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT),
                    Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET)
            .build();

    private final SwerveModule[] modules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

    private final SwerveKinematics swerveKinematics = new SwerveKinematics(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),         //front left
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),        //front right
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),       //back left
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)        //back right
    );

    private final SwerveOdometry swerveOdometry = new SwerveOdometry(swerveKinematics, RigidTransform2.ZERO);

    private final Object sensorLock = new Object();
    private NavX navX$SensorLock = new NavX(SPI.Port.kMXP);

    private final Object kinematicsLock = new Object();
    private RigidTransform2 pose$kinematicsLock = RigidTransform2.ZERO;

    private final Object stateLock = new Object();
    private HolonomicDriveSignal driveSignal$stateLock = null;

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose$kinematicsLock;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal$stateLock = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            navX$SensorLock.setAdjustmentAngle(angle);
        }
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for(int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        synchronized (sensorLock) {
            angle = navX$SensorLock.getAngle();
        }

        RigidTransform2 pose = swerveOdometry.update(angle, dt, moduleVelocities);
        synchronized (kinematicsLock) {
            pose$kinematicsLock = pose;
        }
    }

    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisVelocity chassisVelocity;
        if(driveSignal == null) {
            chassisVelocity = new ChassisVelocity(Vector2.ZERO, 0.0);
        } else if (driveSignal.isFieldOriented()) {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation().rotateBy(getPose().rotation.inverse()),
                    driveSignal.getRotation()
            );
        } else {
            chassisVelocity = new ChassisVelocity(
                    driveSignal.getTranslation(),
                    driveSignal.getRotation()
            );
        }

        Vector2[] moduleOutputs = swerveKinematics.toModuleVelocities(chassisVelocity);
        SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1);
        for(int i = 0; i < moduleOutputs.length; i++) {
            var module = modules[i];
            module.setTargetVelocity(moduleOutputs[i]);
            module.updateState(dt);
        }
    }

    @Override
    public void update(double time, double dt) {
        updateOdometry(dt);

        HolonomicDriveSignal driveSignal;
        synchronized (stateLock) {
            driveSignal = driveSignal$stateLock;
        }

        updateModules(driveSignal, dt);
    }
}
