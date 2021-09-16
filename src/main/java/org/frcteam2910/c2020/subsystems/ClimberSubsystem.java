package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;

public class ClimberSubsystem implements Subsystem {
    private final Solenoid lockSolenoid = new Solenoid(Constants.CLIMBER_LOCK_SOLENOID_PORT);
    private final TalonFX motor = new TalonFX(Constants.CLIMBER_MOTOR_PORT);

    public void unlock() {
        lockSolenoid.set(false);
    }

    public void lock() {
        lockSolenoid.set(true);
    }

    public void setMotorOutput(double output) {
        motor.set(TalonFXControlMode.PercentOutput, output);
    }
}
