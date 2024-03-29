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

public class CubeInstantEject extends InstantCommand implements MustangCommand {
    private CubeIntake cubeIntake;

    public CubeInstantEject(CubeIntake vubeIntake) {
        this.cubeIntake = vubeIntake;
        addRequirements(vubeIntake);
    }

    @Override
    public void initialize() {
        cubeIntake.startEjecting();

    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}