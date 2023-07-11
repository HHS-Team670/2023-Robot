// package frc.team670.robot.commands.pathplanner;

// import java.util.HashMap;
// import java.util.List;
// import java.util.Map;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
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
// import frc.team670.robot.commands.claw.ClawEject;
// import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
// import frc.team670.robot.constants.RobotConstants;
// import frc.team670.robot.subsystems.Claw;
// import frc.team670.robot.subsystems.LED;
// import frc.team670.robot.subsystems.DriveBase;
// import frc.team670.robot.subsystems.arm.Arm;
// import frc.team670.robot.subsystems.arm.ArmState;
// import frc.team670.robot.commands.leds.SetIntakeCone;
// import frc.team670.robot.commands.leds.SetIntakeCube;

// public class CenterEngage extends SequentialCommandGroup implements MustangCommand {

//     String pathName;

//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return new HashMap<MustangSubsystemBase, HealthState>();
//     }


//     public CenterEngage(DriveBase driveBase, Claw claw, Arm arm, LED led, String pathName) {
//         this.pathName = pathName;
//         List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 2.5, 4);

//         HashMap<String, Command> eventMap = new HashMap<>();

//         // eventMap stuff
//         eventMap.put("setIntakeCone", new SetIntakeCone(led,claw));
//         eventMap.put("moveToMid", new MoveToTarget(arm, ArmState.SCORE_MID));
//         eventMap.put("clawEject", new ClawEject(claw));
//         eventMap.put("stow", new MoveToTarget(arm, ArmState.STOWED));
//         eventMap.put("autoLevel", new NonPidAutoLevel(driveBase, true));

//         CommandBase fullAuto = driveBase.getAutoBuilderFromEvents(eventMap).fullAuto(trajectoryGroup);

//         addCommands(fullAuto);
//     }
// }
