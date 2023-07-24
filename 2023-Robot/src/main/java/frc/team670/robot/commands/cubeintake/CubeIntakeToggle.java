package frc.team670.robot.commands.cubeintake;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.CubeIntake;

public class CubeIntakeToggle extends InstantCommand implements MustangCommand {
    CubeIntake cubeIntake;
    Map<MustangSubsystemBase, HealthState> healthRequirements =
                new HashMap<MustangSubsystemBase, HealthState>();


    public CubeIntakeToggle( CubeIntake cubeIntake) {
        this.cubeIntake = cubeIntake;
        addRequirements(cubeIntake);
        healthRequirements.put(cubeIntake, HealthState.YELLOW);

        

    }

    @Override 
    public void execute(){
        if(cubeIntake.getDeployer().isDeployed()){
            cubeIntake.setIdle();
        }else  if(!cubeIntake.isFull()){
            cubeIntake.startIntaking();
        }else{
            cubeIntake.startEjecting();
        }
    }




    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthRequirements;
    }


    
}
