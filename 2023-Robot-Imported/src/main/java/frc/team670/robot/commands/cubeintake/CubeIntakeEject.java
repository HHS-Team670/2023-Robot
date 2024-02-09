package frc.team670.robot.commands.cubeintake;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.commands.MustangCommand;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.robot.subsystems.CubeIntake;

/**
 * @author Tarini, Samanyu and Ishaan
 */

public class CubeIntakeEject extends Command implements MustangCommand {
    private CubeIntake cubeIntake;

    public CubeIntakeEject(CubeIntake cubeIntake) {
        this.cubeIntake = cubeIntake;
        // addRequirements(cubeIntake);
    }

    // @Override
    // public void execute() {
    //     CubeIntake.startEjecting(); //Ejects at default speed. Call startEjecting(double) to set a speed
    // }
    @Override
    public void initialize() {
        cubeIntake.startEjecting();

    }

    @Override
    public boolean isFinished() {
        return !cubeIntake.isFull();
    }

    public void end(boolean interrupted) {
        cubeIntake.setIdle();
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}