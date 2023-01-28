package frc.team670.robot.commands;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
// SparkMAX is used for the motor control.
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.*;
import frc.team670.robot.constants.RobotMap;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Claw.Status;

/**
 * @author Samanyu and Ishaan
 */

public class ClawIdle extends InstantCommand implements MustangCommand 
{
    private Claw claw;

    public ClawIdle(Claw claw)
    {
        this.claw = claw;
    }

    public void initialize() {
        claw.setStatus(Status.OFF);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}