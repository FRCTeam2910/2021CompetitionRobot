package org.frcteam2910.c2020.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.robot.UpdateManager;

public class FeederSubsystem implements Subsystem, UpdateManager.Updatable {

    private TalonFX  motor  = new TalonFX(Constants.FEEDER_MOTOR_PORT);
    private DigitalInput fullSensor = new DigitalInput(Constants.FEEDER_IS_FULL_SENSOR_PORT);
    private DigitalInput intakeBallSensor = new DigitalInput(Constants.FEEDER_INTAKE_BALL_SENSOR_PORT);


    public void spinMotor(double speed){

        motor.set(TalonFXControlMode.PercentOutput,speed);
    }

    public boolean isFull(){
        return fullSensor.get();
    }

    public boolean shouldAdvance(){
        if(isFull()){
            return false;
        }
        return intakeBallSensor.get();
    }

    @Override
    public void update(double time, double dt){



    }

}
