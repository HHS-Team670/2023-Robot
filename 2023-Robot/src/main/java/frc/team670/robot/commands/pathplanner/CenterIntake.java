package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
import frc.team670.robot.commands.claw.ClawInstantIntake;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class CenterIntake extends SequentialCommandGroup implements MustangCommand {
    
    String pathName;

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }


    public CenterIntake(DriveBase driveBase, Claw claw, Arm arm, String pathName) {
        this.pathName = pathName;
        List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 1, 0.5);

        HashMap<String, Command> eventMap = new HashMap<>();

        // eventMap stuff
        eventMap.put("moveToMid", new MoveToTarget(arm, ArmState.SCORE_MID));
        eventMap.put("clawEject", new ClawEject(claw));
        eventMap.put("stow1", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("moveToGround", new MoveToTarget(arm, ArmState.HYBRID));
        eventMap.put("clawIntake", new ClawInstantIntake(claw));
        eventMap.put("autoLevel", new NonPidAutoLevel(driveBase, false));
        
        SwerveDriveKinematics driveBaseKinematics = driveBase.getSwerveKinematics();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(driveBase::getPose,
                driveBase::resetOdometry, driveBaseKinematics, RobotConstants.AUTON_TRANSLATION_CONTROLLER, RobotConstants.AUTON_THETA_CONTROLLER,
                driveBase::setModuleStates, eventMap, true, new Subsystem[] {driveBase});

        CommandBase fullAuto = autoBuilder.fullAuto(trajectoryGroup);

        addCommands(fullAuto);
    }

}
