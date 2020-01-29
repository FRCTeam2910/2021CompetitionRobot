package org.frcteam2910.c2020.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.common.robot.UpdateManager;

public class ClimberSubsystem implements Subsystem, UpdateManager.Updatable {

    private final Solenoid deploySolenoid = new Solenoid(Constants.CLIMBER_DEPLOY_SOLENOID_PORT);
    private final Solenoid extendSolenoid = new Solenoid(Constants.CLIMBER_EXTEND_SOLENOID_PORT);

    private final NetworkTableEntry isDeployedEntry;
    private final NetworkTableEntry isExtendedEntry;

    public ClimberSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        isDeployedEntry = tab.add("Is Climber Deployed", false)
                .withPosition(0, 0)
                .withSize(0, 0)
                .getEntry();
        isExtendedEntry = tab.add("Is Climber Extended", false)
                .withPosition(0, 1)
                .withSize(0, 0)
                .getEntry();
    }

    public void deployClimber(){
        deploySolenoid.set(true);
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

    @Override
    public void periodic() {
        isExtendedEntry.setBoolean(isExtended());
        isDeployedEntry.setBoolean(deploySolenoid.get());
    }
}
