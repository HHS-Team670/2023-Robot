package frc.team670.robot.commands.arm;

import java.util.HashMap;
import java.util.Map;

// import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;


/** 
 * Gets a valid sequence of ArmStates between the current state and the given target state,
 * and queues a SequentialCommandGroup of MoveDirectlyToTarget commands that will only use
 * valid transitions
 */
public class MoveToTarget extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    ArmState target;
    Arm arm;
    public MoveToTarget(ArmState target,Arm arm) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.arm = arm;
        this.target = target;
        //get the valid paths from current state to the target, and add them in sequence
        //call get Valid Path in the arm
        // then call move directly to target for each of those returned paths
        ArmState[] path = Arm.getValidPath(arm.getCurrentState(), target);
        for(int i = 0; i<path.length; i++){
            addCommands(new MoveDirectlyToTarget(arm, path[i]));
        }
    }

  


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}