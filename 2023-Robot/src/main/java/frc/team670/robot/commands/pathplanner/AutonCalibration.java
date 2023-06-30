package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.commands.leds.SetIntakeCone;
import frc.team670.robot.commands.leds.SetIntakeCube;

public class AutonCalibration extends SequentialCommandGroup implements MustangCommand {

    String pathName;

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }


    public AutonCalibration(DriveBase driveBase, String pathName) {
        this.pathName = pathName;
        List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 2.5, 2);

        CommandBase fullAuto = driveBase.getAutoBuilderFromEvents(new HashMap<>()).fullAuto(trajectoryGroup);
        

        addCommands(fullAuto);
    }
}
