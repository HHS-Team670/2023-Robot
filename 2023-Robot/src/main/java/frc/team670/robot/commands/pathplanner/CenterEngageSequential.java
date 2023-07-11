// package frc.team670.robot.commands.pathplanner;

// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
// import frc.team670.robot.commands.arm.MoveToTarget;
// import frc.team670.robot.commands.claw.ClawInstantEject;
// import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
// import frc.team670.robot.commands.drivebase.TurnToAngle;
// import frc.team670.robot.constants.OI;
// import frc.team670.robot.subsystems.Claw;
// import frc.team670.robot.subsystems.LED;
// import frc.team670.robot.subsystems.DriveBase;
// import frc.team670.robot.subsystems.arm.Arm;
// import frc.team670.robot.subsystems.arm.ArmState;
// import frc.team670.robot.commands.leds.SetIntakeCone;
// import frc.team670.robot.commands.leds.SetIntakeCube;

// public class CenterEngageSequential extends SequentialCommandGroup implements MustangCommand {

//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return new HashMap<MustangSubsystemBase, HealthState>();
//     }


//     public CenterEngageSequential(DriveBase driveBase, Claw claw, Arm arm,LED led) {
//         List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup("BackUp", 4, 4.5);
        
//         CommandBase fullAuto = driveBase.getAutoBuilderFromEvents(null).fullAuto(trajectoryGroup);
//         addCommands(new SequentialCommandGroup(new SetIntakeCone(led,claw),new MoveToTarget(arm, ArmState.SCORE_MID),
//                 new ClawInstantEject(claw), new MoveToTarget(arm, ArmState.STOWED), fullAuto,
//                 new TurnToAngle(driveBase, 180, true, OI.getDriverController()),
//                 new NonPidAutoLevel(driveBase, true)));
//     }

// }
