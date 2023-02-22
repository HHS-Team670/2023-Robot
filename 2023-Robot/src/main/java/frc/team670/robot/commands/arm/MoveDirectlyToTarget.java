package frc.team670.robot.commands.arm;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class MoveDirectlyToTarget extends CommandBase implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Arm arm;
    private ArmState end;

    public MoveDirectlyToTarget(Arm arm, ArmState end) {
        addRequirements(arm);
        Logger.consoleLog("Ran MoveDirectlyToTarget with the target " + end.toString());
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(arm, HealthState.GREEN);
        this.arm = arm;
        this.end = end;
    }

    @Override
    public void initialize() {
        arm.moveToTarget(end);
    }

    @Override
    public boolean isFinished() {
        return arm.hasReachedTargetPosition();
    }

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
