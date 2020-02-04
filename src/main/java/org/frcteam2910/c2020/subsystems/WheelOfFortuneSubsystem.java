package org.frcteam2910.c2020.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frcteam2910.c2020.Constants;
import org.frcteam2910.c2020.util.DetectedColor;
import org.frcteam2910.common.robot.UpdateManager;
import edu.wpi.first.wpilibj.util.Color;

public class WheelOfFortuneSubsystem implements Subsystem, UpdateManager.Updatable {

    private final double RED_MIN = Constants.WHEEL_OF_FORTUNE_RED_HUE - (Constants.WHEEL_OF_FORTUNE_RED_HUE - 0) / 2;
    private final double RED_MAX = Constants.WHEEL_OF_FORTUNE_RED_HUE + (Constants.WHEEL_OF_FORTUNE_YELLOW_HUE - Constants.WHEEL_OF_FORTUNE_RED_HUE) / 2;
    private final double GREEN_MIN = Constants.WHEEL_OF_FORTUNE_GREEN_HUE - (Constants.WHEEL_OF_FORTUNE_GREEN_HUE - Constants.WHEEL_OF_FORTUNE_YELLOW_HUE) / 2;
    private final double GREEN_MAX = Constants.WHEEL_OF_FORTUNE_GREEN_HUE + (Constants.WHEEL_OF_FORTUNE_BLUE_HUE - Constants.WHEEL_OF_FORTUNE_GREEN_HUE) / 2;
    private final double BLUE_MIN = Constants.WHEEL_OF_FORTUNE_BLUE_HUE - (Constants.WHEEL_OF_FORTUNE_BLUE_HUE - Constants.WHEEL_OF_FORTUNE_GREEN_HUE) / 2;
    private final double BLUE_MAX = Constants.WHEEL_OF_FORTUNE_BLUE_HUE + (210 - Constants.WHEEL_OF_FORTUNE_BLUE_HUE) / 2;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 COLOR_SENSOR = new ColorSensorV3(i2cPort);


    private DetectedColor detectedColor;
    private final NetworkTableEntry colorEntry;


    public WheelOfFortuneSubsystem(){
        ShuffleboardTab tab = Shuffleboard.getTab("Wheel of Fortune");
        colorEntry = tab.add("Color", DetectedColor.GREEN.toString())
                .withPosition(0,0)
                .withSize(1,1)
                .getEntry();

    }

    @Override
    public void periodic(){
        Color colorFromSensor = COLOR_SENSOR.getColor();
        double hueColor  = calculateHue(colorFromSensor);
        detectedColor = calculateDetectedColor(hueColor);

        colorEntry.setString(detectedColor.toString());
    }


    private double calculateHue(Color colorFromSensor){

        //RGB to Hue formula
        double cMax = Math.max(Math.max(colorFromSensor.red,colorFromSensor.green),colorFromSensor.blue);
        double cMin = Math.min(Math.min(colorFromSensor.red,colorFromSensor.green),colorFromSensor.blue);
        double delta = cMax - cMin;

        if(colorFromSensor.red > colorFromSensor.green && colorFromSensor.red > colorFromSensor.blue){//Red is bigger
             return 60 * (((colorFromSensor.green - colorFromSensor.blue) / delta) % 6);
        }else if(colorFromSensor.green > colorFromSensor.red && colorFromSensor.green > colorFromSensor.blue){//Green is bigger
            return 60 * (((colorFromSensor.blue - colorFromSensor.red) / delta) + 2);
        }else{//Blue is bigger
            return 60 * (((colorFromSensor.red - colorFromSensor.green) / delta) + 4);
        }
    }

    private DetectedColor calculateDetectedColor(double colorHue){
        if(colorHue > RED_MIN && colorHue < RED_MAX){
            return DetectedColor.RED;
        }else if(colorHue > GREEN_MIN && colorHue < GREEN_MAX){
            return DetectedColor.GREEN;
        }else if(colorHue > BLUE_MIN && colorHue < BLUE_MAX){
            return DetectedColor.BLUE;
        }else{
            return DetectedColor.YELLOW;
        }
    }


    public DetectedColor getDetectedColor(){
       return detectedColor;
    }


    @Override
    public void update(double time, double dt){

    }

}
