package frc.team670.robot.commands;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Claw.Status;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class ClawEject extends CommandBase implements MustangCommand 
{
    private Claw claw;
    private int timer;
    
    public ClawEject(Claw claw) {
        this.claw = claw;
        timer = 0;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.setStatus(Status.EJECTING);
        timer++;
    }

    @Override
    public boolean isFinished() {
        // some arbitrary amount of time until ejection is finished
        // TODO: adjust length of time or find a better way to check for finishing ejection
        if (timer > 25) {
            return true;
        } 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        claw.setStatus(Status.IDLE);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}

