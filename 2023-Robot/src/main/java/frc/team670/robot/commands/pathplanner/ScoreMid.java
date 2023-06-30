package frc.team670.robot.commands.pathplanner;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawEject;
import frc.team670.robot.commands.claw.ClawInstantEject;
import frc.team670.robot.commands.claw.ClawInstantIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.commands.leds.SetIntakeCone;
import frc.team670.robot.commands.leds.SetIntakeCube;

public class ScoreMid extends SequentialCommandGroup implements MustangCommand {

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return new HashMap<MustangSubsystemBase, HealthState>();
    }

    public ScoreMid(DriveBase driveBase, Claw claw, Arm arm,LED led) {
        
        addCommands(new SetIntakeCone(led,claw),new ClawInstantIntake(claw),
                    new MoveToTarget(arm, ArmState.SCORE_MID), 
                    new ClawEject(claw), 
                    new MoveToTarget(arm, ArmState.STOWED));
    }

}
