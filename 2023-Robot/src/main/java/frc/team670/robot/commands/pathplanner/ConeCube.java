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
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class ConeCube extends SequentialCommandGroup implements MustangCommand {

    String pathName;

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }


    public ConeCube(DriveBase driveBase, Claw claw, Arm arm, String pathName) {
        this.pathName = pathName;
        List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 2.0, 1.0);

        PIDConstants PID_translation = new PIDConstants(1.0, 0, 0);
        PIDConstants PID_theta = new PIDConstants(1.0, 0, 0); 

        driveBase.resetOdometry(trajectoryGroup.get(0).getInitialHolonomicPose());

        HashMap<String, Command> eventMap = new HashMap<>();

        // eventMap stuff
        eventMap.put("clawIntake1", new ClawIntake(claw));
        eventMap.put("moveToMid1", new MoveToTarget(arm, ArmState.SCORE_MID));
        eventMap.put("clawEject1", new ClawEject(claw));
        eventMap.put("moveToStowed", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("moveToGround", new MoveToTarget(arm, ArmState.HYBRID));
        eventMap.put("clawIntake2", new ClawIntake(claw)); //May want to use IntakeAndStow after testing.
        eventMap.put("moveToStowed2", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("moveToMid2", new MoveToTarget(arm, ArmState.SCORE_MID));
        eventMap.put("clawEject2", new ClawEject(claw));
        eventMap.put("moveToStowed3", new MoveToTarget(arm, ArmState.STOWED));
        
        SwerveDriveKinematics driveBaseKinematics = driveBase.getSwerveKinematics();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(driveBase.getPoseEstimator()::getCurrentPose,
                driveBase::resetOdometry, driveBaseKinematics, PID_translation, PID_theta,
                driveBase::setModuleStates, eventMap, false, new Subsystem[] {driveBase});

        CommandBase fullAuto = autoBuilder.fullAuto(trajectoryGroup);

        addCommands(fullAuto);
    }
}
