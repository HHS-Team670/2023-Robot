package frc.team670.robot.commands.pathplanner;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class MustangAutoBuilder extends CommandBase implements MustangCommand {
    
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    public MustangAutoBuilder() {
            
    }


}
