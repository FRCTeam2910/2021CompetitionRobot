package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

    private static final double GEAR_REDUCTION = 190.0 / 27.0;
    private static final double WHEEL_DIAMETER = 4.0;

    private final SwerveModule frontLeftModule =
            new Mk2SwerveModuleBuilder(new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                    .angleMotor(
                            new CANSparkMax(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                                    CANSparkMaxLowLevel.MotorType.kBrushless),
                            Mk2SwerveModuleBuilder.MotorType.NEO)
                    .driveMotor(
                            new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR), GEAR_REDUCTION, WHEEL_DIAMETER)
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
                            new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR),  GEAR_REDUCTION, WHEEL_DIAMETER)
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
                            new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR), GEAR_REDUCTION, WHEEL_DIAMETER)
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
                            new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR), GEAR_REDUCTION, WHEEL_DIAMETER)
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
    @GuardedBy("sensorLock")
    private NavX navX = new NavX(SPI.Port.kMXP);

    private final Object kinematicsLock = new Object();
    @GuardedBy("kinematicsLock")
    private RigidTransform2 pose = RigidTransform2.ZERO;

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private HolonomicDriveSignal driveSignal = null;

    // Logging
    private final NetworkTableEntry odometryXEntry;
    private final NetworkTableEntry odometryYEntry;
    private final NetworkTableEntry odometryAngleEntry;

    private final NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

    public DrivetrainSubsystem() {
        synchronized (sensorLock) {
            navX.setInverted(true);
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        odometryXEntry = tab.add("X", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        odometryYEntry = tab.add("Y", 0.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        odometryAngleEntry = tab.add("Angle", 0.0)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        ShuffleboardLayout[] moduleLayouts = {
                tab.getLayout("Front Left Module", BuiltInLayouts.kList),
                tab.getLayout("Front Right Module", BuiltInLayouts.kList),
                tab.getLayout("Back Left Module", BuiltInLayouts.kList),
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        };
        for (int i = 0; i < modules.length; i++) {
            ShuffleboardLayout layout = moduleLayouts[i]
                    .withPosition(1 + i * 2, 0)
                    .withSize(2, 4);
            moduleAngleEntries[i] = layout.add("Angle", 0.0).getEntry();
        }
    }

    public RigidTransform2 getPose() {
        synchronized (kinematicsLock) {
            return pose;
        }
    }

    public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean isFieldOriented) {
        synchronized (stateLock) {
            driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented);
        }
    }

    public void resetGyroAngle(Rotation2 angle) {
        synchronized (sensorLock) {
            navX.setAdjustmentAngle(
                    navX.getUnadjustedAngle().rotateBy(angle.inverse())
            );
        }
    }

    private void updateOdometry(double dt) {
        Vector2[] moduleVelocities = new Vector2[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];
            module.updateSensors();

            moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle())).scale(module.getCurrentVelocity());
        }

        Rotation2 angle;
        synchronized (sensorLock) {
            angle = navX.getAngle();
        }

        RigidTransform2 pose = swerveOdometry.update(angle, dt, moduleVelocities);
        synchronized (kinematicsLock) {
            this.pose = pose;
        }
    }

    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisVelocity chassisVelocity;
        if (driveSignal == null) {
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
        for (int i = 0; i < moduleOutputs.length; i++) {
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
            driveSignal = this.driveSignal;
        }

        updateModules(driveSignal, dt);
    }

    @Override
    public void periodic() {
        RigidTransform2 pose = getPose();
        odometryXEntry.setDouble(pose.translation.x);
        odometryYEntry.setDouble(pose.translation.y);
        odometryAngleEntry.setDouble(getPose().rotation.toDegrees());

        for (int i = 0; i < modules.length; i++) {
            moduleAngleEntries[i].setDouble(Math.toDegrees(modules[i].getCurrentAngle()));
        }
    }
}
