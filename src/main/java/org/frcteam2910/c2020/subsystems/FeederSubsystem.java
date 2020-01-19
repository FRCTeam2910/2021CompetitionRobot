package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.robot.UpdateManager;

public class FeederSubsystem implements Subsystem, UpdateManager.Updatable {

    private TalonFX  motor  = new TalonFX(Constants.FEEDER_MOTOR_PORT);



    public void spinMotor(double speed){

        motor.set(TalonFXControlMode.PercentOutput,speed);
    }

    @Override
    public void update(double time, double dt){



    }

}
