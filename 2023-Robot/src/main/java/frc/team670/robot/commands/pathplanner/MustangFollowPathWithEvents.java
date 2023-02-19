package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.drivebase.MustangPPSwerveControllerCommand;

public class MustangFollowPathWithEvents extends FollowPathWithEvents implements MustangCommand {
    
    public MustangFollowPathWithEvents(MustangPPSwerveControllerCommand command, List<EventMarker> eventMarkers, HashMap<String, Command> eventMap) {
        super(command, eventMarkers, eventMap);
    }

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }


}
