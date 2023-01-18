package frc.team670.robot.commands.arm;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.arm.ArmState;


/** 
 * Gets a valid sequence of ArmStates between the current state and the given target state,
 * and queues a SequentialCommandGroup of MoveDirectlyToTarget commands that will only use
 * valid transitions
 */
public class MoveToTarget extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    ArmState target;

    public MoveToTarget(ArmState target) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();

        //get the valid paths from current state to the target, and add them in sequence
        addCommands();
    }

    @Override
    public void initialize() {
        //
    }



    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

}