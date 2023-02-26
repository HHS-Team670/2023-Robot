package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawEject;
import frc.team670.robot.commands.claw.ClawIdle;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Claw.Status;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class ClawToggle extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    public ClawToggle(Claw claw, Arm arm, Status status ){
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        
        healthReqs.put(claw, HealthState.GREEN);
        if(status==Status.EJECTING){
            addCommands(new ClawEject(claw),new MoveToTarget(arm,ArmState.STOWED));
            healthReqs.put(arm, HealthState.GREEN);
        }else if(status==Status.INTAKING){
            if(!claw.isFull()){
                addCommands(new ClawIntake(claw,true),new MoveToTarget(arm,ArmState.STOWED));
                healthReqs.put(arm, HealthState.GREEN);
            }else{
                addCommands(new ClawIntake(claw,false));
            }
            
        }else if(status==Status.IDLE){
            addCommands(new ClawIdle(claw),new MoveToTarget(arm,ArmState.STOWED));
            healthReqs.put(arm, HealthState.GREEN);
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        
        return healthReqs;
    }
}
