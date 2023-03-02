package frc.team670.robot.commands.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.LED;

public class SetLEDsYellow extends InstantCommand{
    LED led;

    public SetLEDsYellow(LED led) {
        //led.setColorYellow();
        this.led = led;
    }

    @Override
    public void initialize(){
        led.setColorYellow();
    }
}