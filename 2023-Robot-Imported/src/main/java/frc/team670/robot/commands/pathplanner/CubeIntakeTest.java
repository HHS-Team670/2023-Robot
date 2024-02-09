package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.cubeintake.CubeIntakeEject;
import frc.team670.robot.commands.cubeintake.CubeIntakeIntake;
import frc.team670.robot.commands.cubeintake.ToggleCubeIntakeDeployer;
import frc.team670.robot.subsystems.CubeIntake;


public class CubeIntakeTest extends SequentialCommandGroup implements MustangCommand {

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }


    public CubeIntakeTest(CubeIntake cubeIntake) {
       
        addCommands(new SequentialCommandGroup(new ToggleCubeIntakeDeployer(cubeIntake),new CubeIntakeIntake(cubeIntake),new CubeIntakeEject(cubeIntake)));
    }

}
