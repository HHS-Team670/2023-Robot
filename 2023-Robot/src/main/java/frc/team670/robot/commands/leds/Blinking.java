package frc.team670.robot.commands.leds;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.LED;

public class Blinking extends InstantCommand implements MustangCommand{

    private LED led;
    HashMap<MustangSubsystemBase, HealthState> healthreqs = new HashMap<MustangSubsystemBase, HealthState>();

    public Blinking(LED led) {
        this.led = led;
        addRequirements(led);
        healthreqs.put(led, HealthState.GREEN);
    }

    public void initialize() {
        led.blinking();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthreqs;
    }
}
