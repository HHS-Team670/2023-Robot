package frc.team670.robot.commands.arm;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.arm.Arm;

public class ResetArmFromAbsolute extends InstantCommand implements MustangCommand {
    private Arm arm;

    public ResetArmFromAbsolute(Arm arm) {
        this.arm = arm;
        
    }

    public void initialize() {
        arm.resetPositionFromAbsolute();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}
