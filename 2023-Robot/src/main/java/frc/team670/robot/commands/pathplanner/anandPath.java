package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import frc.team670.robot.subsystems.drivebase.DriveBase;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.cubeintake.ToggleCubeIntakeDeployer;
import frc.team670.robot.subsystems.CubeIntake;
public class anandPath extends SequentialCommandGroup implements MustangCommand {

    public anandPath(DriveBase driveBase, Arm arm, CubeIntake cubeIntake) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Anand Path", new PathConstraints(4, 3));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("event", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("event1", new ToggleCubeIntakeDeployer(cubeIntake));
        CommandBase fullAuto = driveBase.getAutoBuilderFromEvents(eventMap).fullAuto(pathGroup);
        addCommands(fullAuto);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
