package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.MustangController;

/**
 * MoveToPose - moves to specified pose. Cancels when button is released. 
 */
public class MoveToPose extends CommandBase implements MustangCommand {
    private SwerveDrive swerve;
    private MustangController controller;
    private Pose2d goalPose;
    private PathPlannerTrajectory path;
    protected Map<MustangSubsystemBase, HealthState> healthReqs;

    private MustangScheduler scheduler = MustangScheduler.getInstance();
    private MustangPPSwerveControllerCommand moveCommand = null;

    public MoveToPose(SwerveDrive swerve, Pose2d goalPose) {
        this.swerve = swerve;
        this.controller = null;
        path = null;
        this.goalPose = goalPose;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(swerve, HealthState.GREEN);
        addRequirements(swerve);
    }

    public MoveToPose(SwerveDrive swerve, Pose2d goalPose, MustangController controller) {
        this.swerve = swerve;
        this.controller = controller;
        path = null;
        this.goalPose = goalPose;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(swerve, HealthState.GREEN);
        addRequirements(swerve);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        path = PathPlanner.generatePath(new PathConstraints(1, 0.5), calcStartPoint(),
                calcEndPoint(goalPose));

        // TODO: TUNE PID CONTROLLERS
        PIDController xController = new PIDController(3, 0, 0);
        PIDController yController = new PIDController(3, 0, 0);
        PIDController thetaController = new PIDController(0.2, 0, 0);

        moveCommand = new MustangPPSwerveControllerCommand(path, swerve::getPose,
                swerve.getSwerveKinematics(), xController, yController, thetaController,
                swerve::setModuleStates, new Subsystem[] {swerve});
        scheduler.schedule(moveCommand, swerve);
    }

    @Override
    public boolean isFinished() {
        return controller == null || controller.getAButtonReleased(); // TODO: change based on oi file command button
    }

    @Override
    public void end(boolean interrupted) {
        if (controller != null) scheduler.cancel(moveCommand);
    }


    private PathPoint calcStartPoint() {
        return new PathPoint(swerve.getPose().getTranslation(), new Rotation2d(),
                swerve.getGyroscopeRotation());
    }

    private PathPoint calcEndPoint(Pose2d targetPose) {
        return new PathPoint(targetPose.getTranslation(), new Rotation2d(),
                swerve.getGyroscopeRotation());
    }
}
