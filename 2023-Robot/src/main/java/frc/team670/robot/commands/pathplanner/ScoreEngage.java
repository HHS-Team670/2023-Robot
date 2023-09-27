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
import frc.team670.robot.commands.claw.ClawInstantEject;
import frc.team670.robot.commands.claw.ClawInstantIntake;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
import frc.team670.robot.commands.drivebase.TurnToAngle;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import frc.team670.robot.subsystems.drivebase.DriveBase;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.subsystems.CubeIntake;
import frc.team670.robot.commands.leds.SetIntakeCone;
import frc.team670.robot.commands.leds.SetIntakeCube;
import frc.team670.robot.commands.cubeintake.CubeIntakeIntake;
import frc.team670.robot.commands.cubeintake.CubeIntakeToggle;
import frc.team670.robot.commands.cubeintake.CubeIntakeEject;


public class ScoreEngage extends SequentialCommandGroup implements MustangCommand {

    String pathName;

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }


    public ScoreEngage(DriveBase driveBase, Claw claw, Arm arm,LED led, CubeIntake cubeIntake, String pathName) {
        this.pathName = pathName;
        List<PathPlannerTrajectory> trajectoryGroup = PathPlanner.loadPathGroup(pathName, 1.5, 1);


        HashMap<String, Command> eventMap = new HashMap<>();

        // eventMap stuff
        eventMap.put("setIntakeCone", new SetIntakeCone(led,claw));//here
        eventMap.put("moveToStowed", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("moveToMid", new MoveToTarget(arm, ArmState.SCORE_MID));
        eventMap.put("clawEject", new ClawEject(claw));
        eventMap.put("moveToStowed1", new MoveToTarget(arm, ArmState.STOWED));
        eventMap.put("setIntakeCube", new SetIntakeCube(led,claw));//here
        eventMap.put("moveToGround", new MoveToTarget(arm, ArmState.HYBRID));
        eventMap.put("clawIntake", new ClawInstantIntake(claw)); // May want to use IntakeAndStow
        eventMap.put("cubeIntakeToggle", new CubeIntakeToggle(cubeIntake));
        eventMap.put("cubeIntakeIntake", new CubeIntakeIntake(cubeIntake));
        eventMap.put("cubeIntakeEject", new CubeIntakeEject(cubeIntake));
        CommandBase fullAuto = driveBase.getAutoBuilderFromEvents(eventMap).fullAuto(trajectoryGroup);

        addCommands(fullAuto, new NonPidAutoLevel(driveBase, false));
    }
}
