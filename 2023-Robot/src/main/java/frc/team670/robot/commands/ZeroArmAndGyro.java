package frc.team670.robot.commands;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.arm.ZeroArm;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.arm.Arm;

public class ZeroArmAndGyro extends ParallelCommandGroup implements MustangCommand {

    public ZeroArmAndGyro(DriveBase driveBase, Arm arm) {
        addCommands(new SetSwerveForwardDirection(driveBase), new ZeroArm(arm));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
    
}
