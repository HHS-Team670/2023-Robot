package frc.team670.robot.commands.routines;

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

    public MoveArm(Arm arm, Claw claw, ArmState target){
        addCommands(new MoveToTarget(arm,target));
        addCommands(new ClawToggle(claw,arm,Status.INTAKING));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getHealthRequirements'");
    }
    
}
