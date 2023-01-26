package frc.team670.robot.commands.pathplanner;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class PrintOnStopPoint extends InstantCommand implements MustangCommand {
    
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    public void execute() {
        System.out.println("Waypoint reached");
        SmartDashboard.putBoolean("Waypoint reached", true);
    }

}
