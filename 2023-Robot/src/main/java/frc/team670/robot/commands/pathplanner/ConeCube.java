package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class ConeCube extends SequentialCommandGroup implements MustangCommand {
    
    String pathName;

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap();
    }


    public ConeCube(DriveBase driveBase, String pathName) {
        this.pathName = pathName;
        List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 1.0, 0.5);
        
        PIDConstants PID_translation = new PIDConstants(1.0, 0, 0);
        PIDConstants PID_theta = new PIDConstants(1.0, 0, 0);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("dropOff1", new PrintCommand("Drop Off 1 Occured"));
        eventMap.put("pickup", new PrintCommand("Pickup Occured"));
        eventMap.put("dropOff2", new PrintCommand("Drop Off 2 Occured"));

        // HashMap<String, Command> eventMap = initialzeEventMap(); 

        SwerveDriveKinematics driveBaseKinematics = driveBase.getSwerveKinematics();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveBase::getPose, 
            driveBase::resetOdometry,
            driveBaseKinematics, 
            PID_translation, 
            PID_theta,
            driveBase::setModuleStates, 
            eventMap,
            false,
            new Subsystem[] {driveBase}
        );

        CommandBase fullAuto = autoBuilder.fullAuto(trajectoryGroup);

        addCommands(fullAuto);
    }

    // not exactly needed right now bc markers and events are the same for both sides (maybe utilized later)

    // public HashMap<String, Command> initialzeEventMap() {
    //     HashMap<String, Command> eventMap = new HashMap<>();
    //     if (pathName.equals("LeftConeCube")) {
    //         eventMap.put("dropOff1", new PrintCommand("Drop Off 1 Occured"));
    //         eventMap.put("pickup", new PrintCommand("Pickup Occured"));
    //         eventMap.put("dropOff2", new PrintCommand("Drop Off 2 Occured"));
    //     }
    //     else if (pathName.equals("RightConeCube")) {
    //         eventMap.put("dropOff1", new PrintCommand("Drop Off 1 Occured"));
    //         eventMap.put("pickup", new PrintCommand("Pickup Occured"));
    //         eventMap.put("dropOff2", new PrintCommand("Drop Off 2 Occured"));
    //     }
    //     return eventMap;
    // }
}
