package frc.team670.robot.commands.drivebase;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class MustangPPSwerveControllerCommand extends PPSwerveControllerCommand implements MustangCommand {

    public MustangPPSwerveControllerCommand(PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier,
            SwerveDriveKinematics kinematics, PIDController xController, PIDController yController,
            PIDController rotationController, Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem[] requirements) {
        super(trajectory, poseSupplier, kinematics, xController, yController, rotationController, outputModuleStates,
                requirements);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
    
}