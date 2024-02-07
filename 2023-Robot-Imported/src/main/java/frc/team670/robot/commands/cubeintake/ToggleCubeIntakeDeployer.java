package frc.team670.robot.commands.cubeintake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.CubeIntake;

public class ToggleCubeIntakeDeployer extends InstantCommand implements MustangCommand {
    CubeIntake cubeIntake;
    Map<MustangSubsystemBase, HealthState> healthRequirements =
                new HashMap<MustangSubsystemBase, HealthState>();


    public ToggleCubeIntakeDeployer( CubeIntake cubeIntake) {
        this.cubeIntake = cubeIntake;
        healthRequirements.put(cubeIntake, HealthState.YELLOW);
    }

    @Override 
    public void execute(){
        // cubeIntake.startEjecting();
        cubeIntake.toggleDeployer();

    }




    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthRequirements;
    }


    
}
