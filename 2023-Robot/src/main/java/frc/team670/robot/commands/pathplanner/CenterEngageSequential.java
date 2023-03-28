package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.swervelib.pathplanner.MustangFollowPathWithEvents;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawInstantEject;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
import frc.team670.robot.commands.drivebase.TurnToAngle;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class CenterEngageSequential extends SequentialCommandGroup implements MustangCommand {
    
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {

        return new HashMap<MustangSubsystemBase, HealthState>();
    }


    public CenterEngageSequential(DriveBase driveBase, Claw claw, Arm arm) {
        SwerveDriveKinematics driveBaseKinematics = driveBase.getSwerveKinematics();
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(driveBase::getPose,
                driveBase::resetOdometry, driveBaseKinematics, RobotConstants.AUTON_TRANSLATION_CONTROLLER, RobotConstants.AUTON_THETA_CONTROLLER,
                driveBase::setModuleStates, null, true, new Subsystem[] {driveBase});
                
        addCommands(new SequentialCommandGroup(
            new MoveToTarget(arm, ArmState.SCORE_MID),
            new ClawInstantEject(claw),
            new MoveToTarget(arm, ArmState.STOWED),
            autoBuilder.followPath(PathPlanner.loadPath("BackUp", new PathConstraints(2, 1))),
            new TurnToAngle(driveBase, 180, true, OI.getDriverController()),
            new NonPidAutoLevel(driveBase, true)));
    }

}
