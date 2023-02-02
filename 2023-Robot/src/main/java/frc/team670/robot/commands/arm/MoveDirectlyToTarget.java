package frc.team670.robot.commands.arm;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import java.util.*;

/*
 * Sets new target state and runs untill reach that state checks isAt
 */
public class MoveDirectlyToTarget extends CommandBase implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Arm arm;
   
    private ArmState end;

    public MoveDirectlyToTarget(Arm arm, ArmState end){
        //
        addRequirements(arm);
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.arm = arm;
        
        this.end = end;
    }

    @Override
    public void initialize()
    {
        arm.moveToTarget(end);
    }

    @Override
    public boolean isFinished()
    {
        return arm.isAt(end);
    }

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
