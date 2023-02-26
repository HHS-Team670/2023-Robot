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

public class ClawEject extends InstantCommand implements MustangCommand {
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
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}