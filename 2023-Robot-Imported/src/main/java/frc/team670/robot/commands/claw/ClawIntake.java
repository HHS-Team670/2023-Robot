package frc.team670.robot.commands.claw;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.Claw;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class ClawIntake extends Command implements MustangCommand {
    private Claw claw;

    public ClawIntake(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    // @Override
    // public void execute() {
    //     claw.startIntaking();
    // }

    @Override
    public void initialize() {
        claw.startIntaking();

    }

    @Override
    public boolean isFinished() {
        return claw.isFull();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}