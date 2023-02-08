package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

/**
 * MoveToPose
 */
public class MoveToPose extends InstantCommand implements MustangCommand {
    private SwerveDrive swerve;
    private boolean isRelative;
    private PathPlannerTrajectory path;
    private MustangScheduler scheduler = MustangScheduler.getInstance();
    private double x, y;
    private Rotation2d rotation;
    protected Map<MustangSubsystemBase, HealthState> healthReqs;


    public MoveToPose(SwerveDrive swerve, double x, double y, Rotation2d rotation,
            boolean isRelative) {
        this.swerve = swerve;
        this.isRelative = isRelative;
        path = null;
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(swerve, HealthState.GREEN);
    }

    public MoveToPose(SwerveDrive swerve, Pose2d targetPose, boolean isRelative) {
        this.swerve = swerve;
        this.isRelative = false;
        path = null;
        this.x = targetPose.getX();
        this.y = targetPose.getY();
        this.rotation = targetPose.getRotation();
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(swerve, HealthState.GREEN);
    }

    private PathPoint calcStartPoint() {
        return new PathPoint(swerve.getPose().getTranslation(), new Rotation2d(),
                swerve.getGyroscopeRotation());
    }

    private PathPoint calcEndPoint(Pose2d targetPose) {
        return new PathPoint(targetPose.getTranslation(), new Rotation2d(),
                swerve.getGyroscopeRotation());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        Pose2d targetPose;
        if (isRelative)
            targetPose = new Pose2d(swerve.getPose().getX() + x, swerve.getPose().getY() + y,
                    swerve.getPose().getRotation().plus(rotation));
        else
            targetPose = new Pose2d(x, y, rotation);

        path = PathPlanner.generatePath(new PathConstraints(1, 0.5), calcStartPoint(),
                calcEndPoint(targetPose));

        // TODO: TUNE PID CONTROLLERS
        PIDController xController = new PIDController(3, 0, 0);
        PIDController yController = new PIDController(3, 0, 0);
        PIDController thetaController = new PIDController(0.2, 0, 0);

        scheduler.schedule(new MustangPPSwerveControllerCommand(path, swerve::getPose,
                swerve.getSwerveKinematics(), xController, yController, thetaController,
                swerve::setModuleStates, new Subsystem[] {swerve}), swerve);
    }
}