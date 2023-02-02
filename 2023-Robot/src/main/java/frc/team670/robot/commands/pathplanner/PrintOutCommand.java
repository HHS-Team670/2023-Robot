package frc.team670.robot.commands.pathplanner;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class PrintOutCommand extends InstantCommand implements MustangCommand {
    
    String message;

    public PrintOutCommand(String message) {
        this.message = message;
    }

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    public void execute() {
        System.out.println(message);
        SmartDashboard.putBoolean(message, true);
    }

}
