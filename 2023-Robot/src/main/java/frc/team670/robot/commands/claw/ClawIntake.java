package frc.team670.robot.commands.claw;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.Claw;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class ClawIntake extends InstantCommand implements MustangCommand {
    private Claw claw;
    private boolean returnToStowed;

    public ClawIntake(Claw claw, boolean returnToStowed) {
        this.claw = claw;
        this.returnToStowed = returnToStowed;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.startIntaking(returnToStowed);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}