package frc.team670.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.LED;

public class SetLEDsPurple extends InstantCommand{
    LED led;


    public SetLEDsPurple(LED led) {
        //led.setColorPurple();
        this.led = led;
    }
    
    @Override
    public void initialize(){
        led.setColorPurple();
    }

    
}