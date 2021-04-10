package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.robot.UpdateManager;


public class IntakeSubsystem implements Subsystem, UpdateManager.Updatable {
    private TalonFX left_intake_motor = new TalonFX(Constants.LEFT_INTAKE_MOTOR_PORT);
    private TalonFX right_intake_motor = new TalonFX(Constants.RIGHT_INTAKE_MOTOR_PORT);
    private Solenoid extensionTopSolenoid = new Solenoid(Constants.TOP_INTAKE_EXTENSION_SOLENOID);
    private Solenoid extensionBottomSolenoid = new Solenoid(Constants.BOTTOM_INTAKE_EXTENSION_SOLENOID);

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private double motorOutput = 0.0;
    @GuardedBy("stateLock")
    private boolean topExtended = false;
    @GuardedBy("stateLock")
    private boolean bottomExtended = false;

    private final NetworkTableEntry leftMotorSpeedEntry;
    private final NetworkTableEntry rightMotorSpeedEntry;
    private final NetworkTableEntry isTopExtendedEntry;
    private final NetworkTableEntry isBottomExtendedEntry;

    public IntakeSubsystem() {
        right_intake_motor.follow(left_intake_motor);
        right_intake_motor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        right_intake_motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

        left_intake_motor.setInverted(true);

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        leftMotorSpeedEntry = tab.add("Left Motor Speed", 0.0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();
        rightMotorSpeedEntry = tab.add("Right Motor Speed",0.0)
                .withPosition(1,0)
                .withSize(1,1)
                .getEntry();
        isTopExtendedEntry = tab.add("Is Top Extended", false)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        isBottomExtendedEntry = tab.add("Is Bottom Extended", false)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry();
    }

    @Override
    public void update(double time, double dt) {
        double localMotorOutput;
        boolean localTopExtended;
        boolean localBottomExtended;
        synchronized (stateLock) {
            localMotorOutput = motorOutput;
            localTopExtended = topExtended;
            localBottomExtended = bottomExtended;
        }

        left_intake_motor.set(ControlMode.PercentOutput, localMotorOutput);
        if (localTopExtended != extensionTopSolenoid.get()) {
            extensionTopSolenoid.set(localTopExtended);
        }

        extensionBottomSolenoid.set(localBottomExtended);

    }

    public boolean isTopExtended() {
        synchronized (stateLock) {
            return topExtended;
        }
    }

    public void setTopExtended(boolean topExtended) {
        synchronized (stateLock) {
            this.topExtended = topExtended;
        }
    }

    public void setMotorOutput(double motorOutput) {
        synchronized (stateLock) {
            this.motorOutput = motorOutput;
        }
    }

    public double getLeftMotorOutput() {
        synchronized (stateLock) {
            return left_intake_motor.getMotorOutputPercent();
        }
    }

    public double getRightMotorOutput(){
        synchronized (stateLock){
            return right_intake_motor.getMotorOutputPercent();
        }
    }

    public void setBottomExtended(boolean extended){
        synchronized (stateLock){
            bottomExtended = extended;
        }
    }

    public boolean isBottomExtended(){
        synchronized (stateLock){
            return bottomExtended;
        }
    }

    @Override
    public void periodic() {
        leftMotorSpeedEntry.setDouble(getLeftMotorOutput());
        rightMotorSpeedEntry.setDouble(getRightMotorOutput());
        isTopExtendedEntry.setBoolean(extensionTopSolenoid.get());
        isBottomExtendedEntry.setBoolean(extensionBottomSolenoid.get());
    }
}
