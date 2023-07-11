// package frc.team670.robot.commands.pathplanner;

// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
// import frc.team670.robot.commands.arm.MoveToTarget;
// import frc.team670.robot.commands.claw.ClawInstantEject;
// import frc.team670.robot.commands.claw.ClawInstantIntake;
// import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
// import frc.team670.robot.commands.drivebase.TurnToAngle;
// import frc.team670.robot.constants.OI;
// import frc.team670.robot.constants.RobotConstants;
// import frc.team670.robot.subsystems.Claw;
// import frc.team670.robot.subsystems.DriveBase;
// import frc.team670.robot.subsystems.arm.Arm;
// import frc.team670.robot.subsystems.arm.ArmState;
// import frc.team670.robot.subsystems.LED;
// import frc.team670.robot.commands.leds.SetIntakeCone;
// import frc.team670.robot.commands.leds.SetIntakeCube;

// public class ScoreEngage extends SequentialCommandGroup implements MustangCommand {

//     String pathName;

//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return new HashMap<MustangSubsystemBase, HealthState>();
//     }


//     public ScoreEngage(DriveBase driveBase, Claw claw, Arm arm,LED led, String pathName) {
//         this.pathName = pathName;
//         List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 4, 4.5);


//         HashMap<String, Command> eventMap = new HashMap<>();

//         // eventMap stuff
//         eventMap.put("setIntakeCone", new SetIntakeCone(led,claw));//here
//         eventMap.put("moveToMid", new MoveToTarget(arm, ArmState.SCORE_MID));
//         eventMap.put("moveToStowed", new MoveToTarget(arm, ArmState.STOWED));
//         eventMap.put("clawEject", new ClawInstantEject(claw));
//         eventMap.put("setIntakeCube", new SetIntakeCube(led,claw));//here
//         eventMap.put("moveToGround", new MoveToTarget(arm, ArmState.HYBRID));
//         eventMap.put("clawIntake", new ClawInstantIntake(claw)); // May want to use IntakeAndStow
        
//         CommandBase fullAuto = driveBase.getAutoBuilderFromEvents(eventMap).fullAuto(trajectoryGroup);

//         addCommands(fullAuto, new TurnToAngle(driveBase, 180, true, OI.getDriverController()),
//                 new NonPidAutoLevel(driveBase, true));
//     }
// }
