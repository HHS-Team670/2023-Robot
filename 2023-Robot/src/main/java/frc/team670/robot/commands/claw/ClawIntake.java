package frc.team670.robot.commands.claw;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Claw.Status;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class ClawIntake extends InstantCommand implements MustangCommand 
{
    private Claw claw;

    public ClawIntake(Claw claw)
    {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.setStatus(Status.INTAKING);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}