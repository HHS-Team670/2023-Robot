package frc.team670.robot.commands.claw;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.subsystems.Claw;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class ClawEject extends CommandBase implements MustangCommand {
    private Claw claw;

    public ClawEject(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute() {
        claw.startEjecting(); //Ejects at default speed. Call startEjecting(double) to set a speed
    }

    @Override
    public boolean isFinished() {
        return !claw.isFull();
    }

    public void end(boolean interrupted) {
        claw.setIdle();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}