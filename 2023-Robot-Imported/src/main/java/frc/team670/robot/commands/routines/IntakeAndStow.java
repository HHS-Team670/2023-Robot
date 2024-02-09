package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class IntakeAndStow extends SequentialCommandGroup implements MustangCommand {
    private Map<MustangSubsystemBase, HealthState> healthReqs;
    private Claw claw;
    private Arm arm;

    public IntakeAndStow(Claw claw, Arm arm) {
        healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        healthReqs.put(arm, HealthState.GREEN);
        addRequirements(arm, claw);
        this.arm = arm;
        this.claw = claw;
        addCommands(new ClawIntake(claw), new MoveToTarget(arm, claw, ArmState.STOWED));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }
}
