package frc.team670.robot.commands.claw;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class ClawEject extends CommandBase implements MustangCommand 
{
    private Claw claw;
    private Arm arm;
    private int timer;
    
    public ClawEject(Claw claw, Arm arm) {
        this.claw = claw;
        this.arm = arm;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        timer = 0;
    }

    @Override
    public void execute() {
        claw.startEjecting(RobotConstants.CLAW_EJECTING_SPEED);
        timer++;
    }

    @Override
    public boolean isFinished() {
        return timer > 25;
    }

    @Override
    public void end(boolean interrupted) {
        claw.setIdle();
        MustangScheduler.getInstance().schedule(new MoveToTarget(arm, ArmState.STOWED));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}