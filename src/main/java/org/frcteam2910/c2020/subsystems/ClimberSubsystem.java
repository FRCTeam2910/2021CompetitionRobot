package org.frcteam2910.c2020.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.robot.UpdateManager;

public class ClimberSubsystem implements Subsystem, UpdateManager.Updatable {

    private final Solenoid deploySolenoid = new Solenoid(Constants.CLIMBER_DEPLOY_SOLENOID_PORT);
    private final Solenoid extendSolenoid = new Solenoid(Constants.CLIMBER_EXTEND_SOLENOID_PORT);


    public void deployClimber(){
        deploySolenoid.set(false);
    }

    public void extendClimber(){
        extendSolenoid.set(true);
    }

    public boolean isExtended(){
        return extendSolenoid.get();
    }

    public void retractClimber(){
        extendSolenoid.set(false);
    }

    @Override
    public void update(double time, double dt){

    }

}
