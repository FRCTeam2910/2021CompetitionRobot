package org.frcteam2910.c2020.subsystems;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.common.robot.UpdateManager;

import static org.frcteam2910.c2020.Constants.INTAKE_EXTENSION_SOLENOID;
import static org.frcteam2910.c2020.Constants.INTAKE_MOTOR;

public class IntakeSubsystem implements Subsystem, UpdateManager.Updatable {
    private CANSparkMax motor = new CANSparkMax(INTAKE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    private Solenoid extensionSolenoid = new Solenoid(INTAKE_EXTENSION_SOLENOID);

    private final Object stateLock = new Object();
    @GuardedBy("stateLock")
    private double motorOutput = 0.0;
    @GuardedBy("stateLock")
    private boolean extended = false;

    @Override
    public void update(double time, double dt) {
        double localMotorOutput;
        boolean localExtended;
        synchronized (stateLock) {
            localMotorOutput = motorOutput;
            localExtended = extended;
        }

        motor.set(localMotorOutput);
        if (localExtended != extensionSolenoid.get()) {
            extensionSolenoid.set(localExtended);
        }
    }

    public boolean isExtended() {
        synchronized (stateLock) {
            return extended;
        }
    }

    public void setExtended(boolean extended) {
        synchronized (stateLock) {
            this.extended = extended;
        }
    }

    public void setMotorOutput(double motorOutput) {
        synchronized (stateLock) {
            this.motorOutput = motorOutput;
        }
    }
}
