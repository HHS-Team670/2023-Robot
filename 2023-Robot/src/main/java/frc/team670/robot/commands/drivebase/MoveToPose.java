package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.RobotConstants;

/**
 * MoveToPose
 */
public class MoveToPose extends InstantCommand implements MustangCommand {
    private SwerveDrive swerve;
    private boolean isRelative;
    private PathPlannerTrajectory path;
    private MustangController controller;
    private double x, y;
    private MustangScheduler scheduler = MustangScheduler.getInstance();

    protected Map<MustangSubsystemBase, HealthState> healthReqs;


    public MoveToPose(SwerveDrive swerve, double x, double y, boolean isRelative,
            MustangController controller) {
        this.x = x;
        this.y = y;
        this.controller = controller;
        this.swerve = swerve;
        this.isRelative = isRelative;

        path = null;

        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(swerve, HealthState.GREEN);
    }

    private PathPoint calcStartPoint() {
        return new PathPoint(swerve.getPose().getTranslation(), swerve.getGyroscopeRotation());
    }

    private PathPoint calcEndPoint() {
        Pose2d targetPose;
        if (isRelative) {
            targetPose = new Pose2d(swerve.getPose().getX() + x, swerve.getPose().getY() + y, null);
        } else {
            targetPose = new Pose2d(x, y, swerve.getGyroscopeRotation());
        }
        return new PathPoint(targetPose.getTranslation(), targetPose.getRotation());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        path = PathPlanner.generatePath(new PathConstraints(0.5, 0.1), calcStartPoint(),
                calcEndPoint());

        // TODO: TUNE PID CONTROLLERS
        PIDController xController = new PIDController(0.5, 0, 0);
        PIDController yController = new PIDController(0.5, 0, 0);
        PIDController θController = new PIDController(4, 0, 1);
        
        scheduler.schedule(new MustangPPSwerveControllerCommand(path, swerve::getPose,
                swerve.getSwerveKinematics(), xController, yController, θController, swerve::setModuleStates, new Subsystem[] {swerve}), swerve);
    }
}
