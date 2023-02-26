package frc.team670.robot.commands.claw;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Claw.Status;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class ClawIntake extends CommandBase implements MustangCommand {
    private Claw claw;
    private boolean waitTillFull;
    public ClawIntake(Claw claw,boolean waitTillFull) {
        this.claw = claw;
        this.waitTillFull=waitTillFull;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
        claw.setStatus(Status.INTAKING);
    }
    @Override
    public boolean isFinished(){
        return !waitTillFull||claw.isFull();
    }
    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}