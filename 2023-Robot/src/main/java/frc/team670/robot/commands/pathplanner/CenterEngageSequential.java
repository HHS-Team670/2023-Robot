package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawInstantEject;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
import frc.team670.robot.commands.drivebase.TurnToAngle;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class CenterEngageSequential extends SequentialCommandGroup implements MustangCommand {
    
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }


    public CenterEngageSequential(DriveBase driveBase, Claw claw, Arm arm) {
        addCommands(new SequentialCommandGroup(
            new MoveToTarget(arm, ArmState.SCORE_MID),
            new ClawInstantEject(claw),
            new TurnToAngle(driveBase, 180),
            new NonPidAutoLevel(driveBase, true)));
    }

}
