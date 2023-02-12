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
import frc.team670.robot.commands.arm.MoveDirectlyToTarget;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawEject;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;

public class CubeEngage extends SequentialCommandGroup implements MustangCommand {
    
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap();
    }

    public CubeEngage(DriveBase driveBase, Claw claw, Arm arm, String pathName) {
        List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 3, 1.5);
        
        PIDConstants PID_translation = new PIDConstants(1.0, 0, 0);
        PIDConstants PID_theta = new PIDConstants(1.0, 0, 0);

        Map<String, Command> eventMap = new HashMap<>();

        // eventMap stuff
        eventMap.put("moveToHigh", new MoveToTarget(arm, ArmState.SCORE_HIGH));
        eventMap.put("clawEject", new ClawEject(claw));
        eventMap.put("moveToStowed1", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("moveToBackwards", new MoveToTarget(arm, ArmState.BACKWARD_GROUND));
        eventMap.put("clawIntake", new ClawIntake(claw));
        eventMap.put("moveToStowed2", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("autoLevel", new NonPidAutoLevel(driveBase)); // regardless of what side (right/left) you are on, markers are the same

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
