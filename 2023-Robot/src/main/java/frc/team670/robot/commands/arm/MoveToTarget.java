package frc.team670.robot.commands.arm;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;


/** 
 * Gets a valid sequence of ArmStates between the current state and the given target state,
 * and queues a SequentialCommandGroup of MoveDirectlyToTarget commands that will only use
 * valid transitions
 */
public class MoveToTarget extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private ArmState target;
    private Arm arm;

    public MoveToTarget(Arm arm, ArmState target) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(arm, HealthState.GREEN);
        this.arm = arm;
        this.target = target;

        Logger.consoleLog("Ran MoveToTarget with the target " + target.toString());
        // 1) get the valid paths from current state to the target, and add them in sequence
        // 2) call get Valid Path in the arm
        // 3) then call move directly to target for each of those returned paths
        ArmState[] path = Arm.getValidPath(arm.getTargetState(), target);
        for(int i = 0; i<path.length; i++){
            addCommands(new MoveDirectlyToTarget(arm, path[i]));
        }
    }



    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}