package frc.team670.robot.commands.arm;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class ZeroArm extends CommandBase implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Arm arm;

    public ZeroArm(Arm arm) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.arm=arm;
        addRequirements(arm);
        //Move arm straight up
        // Substract by 0.5 rotations(Straight down)
        //Set that to 0
        //Set offset
        // Save that set relative encoders /- offsets
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}