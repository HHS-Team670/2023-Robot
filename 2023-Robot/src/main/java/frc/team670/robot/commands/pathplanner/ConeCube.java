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

public class LeftConeCube extends SequentialCommandGroup implements MustangCommand {
    
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap();
    }

    public LeftConeCube(DriveBase driveBase) {
        List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup("LeftConeCube", 1.0, 0.5);
        
        PIDConstants PID_translation = new PIDConstants(1.0, 0, 0);
        PIDConstants PID_theta = new PIDConstants(1.0, 0, 0);

        Map<String, Command> eventMap = new HashMap<>();
        eventMap.put("event", new PrintOnStopPoint());

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