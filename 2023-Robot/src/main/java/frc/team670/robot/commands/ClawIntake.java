package frc.team670.robot.commands;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Claw.Status;
import frc.team670.robot.constants.RobotConstants;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class ClawIntake extends CommandBase implements MustangCommand 
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
    public boolean isFinished() {
        if (claw.getLeftCurrent() >= RobotConstants.CURRENT_MAX && claw.getRightCurrent() >= RobotConstants.CURRENT_MAX) {
            return true;
        } else {
            return false;
        }
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