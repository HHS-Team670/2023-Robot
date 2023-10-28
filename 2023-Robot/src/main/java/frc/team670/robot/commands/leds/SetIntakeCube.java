package frc.team670.robot.commands.leds;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

import frc.team670.robot.subsystems.LED;


import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;

//our claw has to change intake direction for cone and cube intakes all references to the claw have been removed in this class
public class SetIntakeCube extends InstantCommand implements MustangCommand {
    private LED led;
    
    HashMap<MustangSubsystemBase, HealthState> healthreqs = new HashMap<MustangSubsystemBase, HealthState>();

    public SetIntakeCube(LED led) {
        this.led = led;
        
        addRequirements(led);
        healthreqs.put(led, HealthState.GREEN);
    }

    public void initialize() {
        led.setCubeColor();
        
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthreqs;
    }

}
