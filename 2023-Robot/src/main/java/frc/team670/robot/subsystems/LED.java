package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.mustanglib.subsystems.LEDSubsystem;
import frc.team670.mustanglib.utils.LEDColor;
//import frc.team670.robot.RobotContainer;

/**
 * Represents the LED light strips on the robot.
 * These LEDs change color depending on the selected autonomous path, the number of cargo in the conveyor, and the shooting status.
 * 
 * @author AkshatAdsule, LakshBhambhani, Anomalous-0
 */

 public class LED extends LEDSubsystem {
    
    
    public LED(int port, int length) {
        super(port, length);
        //TODO Auto-generated constructor stub
    }

    private LEDColor color;


    public void setColorPurple(LEDColor color){
        solid(LEDColor.PURPLE.dimmer());
    }
    public void setColorYellow(LEDColor color){
        solid(LEDColor.YELLOW.dimmer());
    }
}

