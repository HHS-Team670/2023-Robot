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
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawEject;
import frc.team670.robot.commands.claw.ClawInstantEject;
import frc.team670.robot.commands.claw.ClawInstantIntake;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
import frc.team670.robot.constants.RobotConstants;

public class CubeEngage extends SequentialCommandGroup implements MustangCommand {

    String pathName;

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<>();
    }

    public CubeEngage(DriveBase driveBase, Claw claw, Arm arm, String pathName) {
        this.pathName = pathName;
        List<PathPlannerTrajectory> trajectoryGroup =
                PathPlanner.loadPathGroup(pathName, 2.75, 1.5);

        PIDConstants PID_translation = RobotConstants.DriveBase.kAutonTranslationPID;
        PIDConstants PID_theta = RobotConstants.DriveBase.kAutonThetaPID;

        Map<String, Command> eventMap = new HashMap<>();

        eventMap.put("moveToMid", new MoveToTarget(arm, ArmState.SCORE_MID));
        eventMap.put("clawEject", new ClawInstantEject(claw));
        eventMap.put("clawIntake2", new ClawInstantIntake(claw));
        eventMap.put("moveToStowed", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("moveToGround", new MoveToTarget(arm, ArmState.HYBRID));
        eventMap.put("moveToStowed2", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("autoLevel", new NonPidAutoLevel(driveBase, false)); // regardless of what side
                                                                          // (right/left) you are
                                                                          // on, markers are the
                                                                          // same

        SwerveDriveKinematics driveBaseKinematics = driveBase.getSwerveKinematics();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(driveBase::getPose,
                driveBase::resetOdometry, driveBaseKinematics, PID_translation, PID_theta,
                driveBase::setModuleStates, eventMap, true, new Subsystem[] {driveBase});

        CommandBase fullAuto = autoBuilder.fullAuto(trajectoryGroup);

        addCommands(fullAuto);
    }
}
