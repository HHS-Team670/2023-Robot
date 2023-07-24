package frc.team670.robot.commands.routines;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.cubeintake.CubeIntakeIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.CubeIntake;
import frc.team670.robot.subsystems.arm.Arm;


public class DualIntake extends ParallelCommandGroup implements MustangCommand{
    CubeIntake cubeIntake;
    Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();

    public  DualIntake (Claw claw, Arm arm, CubeIntake cubeIntake){
        this.cubeIntake = cubeIntake;
        healthRequirements.put(cubeIntake, HealthState.YELLOW);
        addCommands(new IntakeAndStow(claw, arm), new CubeIntakeIntake(cubeIntake));
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
            return healthRequirements;
    }
    
}
