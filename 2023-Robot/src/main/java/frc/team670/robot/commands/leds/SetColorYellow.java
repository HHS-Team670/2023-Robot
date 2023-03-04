package frc.team670.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.LED;

public class SetColorYellow extends InstantCommand {
    LED led;

    public SetColorYellow(LED led) {
        this.led = led;
    }

    @Override
    public void initialize() {
        led.setColorYellow();
    }

}