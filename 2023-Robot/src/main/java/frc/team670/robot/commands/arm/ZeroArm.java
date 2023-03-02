package frc.team670.robot.commands.arm;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.arm.Arm;

public class ZeroArm extends InstantCommand implements MustangCommand {
    Arm arm;

    public ZeroArm(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {
        arm.resetPositionFromAbsolute();
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
