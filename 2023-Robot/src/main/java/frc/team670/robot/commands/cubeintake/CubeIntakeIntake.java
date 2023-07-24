package frc.team670.robot.commands.cubeintake;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.robot.subsystems.CubeIntake;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class CubeIntakeIntake extends CommandBase implements MustangCommand {
    private CubeIntake cubeIntake;

    public CubeIntakeIntake(CubeIntake cubeIntake) {
        this.cubeIntake = cubeIntake;
        addRequirements(cubeIntake);
    }

    // @Override
    // public void execute() {
    //     CubeIntake.startIntaking();
    // }

    @Override
    public void initialize() {
        cubeIntake.startIntaking();

    }

    @Override
    public boolean isFinished() {
        return cubeIntake.isFull();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}