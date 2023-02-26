package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Claw.Status;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class MoveArm extends ParallelCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;

    public MoveArm(Arm arm, Claw claw, ArmState target) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(arm, HealthState.GREEN);
        healthReqs.put(claw, HealthState.GREEN);
        addCommands(new MoveToTarget(arm, target));
        addCommands(new ClawToggle(claw, arm, Status.INTAKING));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}
