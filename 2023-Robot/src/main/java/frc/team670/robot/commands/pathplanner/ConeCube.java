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

    public HashMap<String, Command> initialzeEventMap() {
        HashMap<String, Command> eventMap = new HashMap<>();
        if (pathName.equals("LeftConeCube")) {
            eventMap.put("dropOff1", new PrintOutCommand("Drop Off 1 Occured"));
            eventMap.put("pickup", new PrintOutCommand("Pickup Occured"));
            eventMap.put("dropOff2", new PrintOutCommand("Drop Off 2 Occured"));
        }

        return eventMap;
    }

    public ConeCube(DriveBase driveBase, String pathName) {
        this.pathName = pathName;
        List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 1.0, 0.5);
        
        PIDConstants PID_translation = new PIDConstants(1.0, 0, 0);
        PIDConstants PID_theta = new PIDConstants(1.0, 0, 0);

        
        HashMap<String, Command> eventMap = initialzeEventMap(); 

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
}
