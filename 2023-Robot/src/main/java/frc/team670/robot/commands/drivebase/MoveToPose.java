package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * MoveToPose - moves to specified pose. Cancels when button is released.
 */
public class MoveToPose extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    private final Pose2d BLUE_RELATIVE_END_POSE;
    private Pose2d endPose;
    private Pose2d startPose;
    private boolean backOut = false;

    protected Map<MustangSubsystemBase, HealthState> healthReqs;

    private MustangPPSwerveControllerCommand pathDrivingCommand;

    public MoveToPose(DriveBase driveBase, Pose2d blueRelativeEndPose) {
        this.driveBase = driveBase;
        this.BLUE_RELATIVE_END_POSE = blueRelativeEndPose;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
    }

    public MoveToPose(DriveBase driveBase, Pose2d blueRelativeEndPose, boolean backOut) {
        this.driveBase = driveBase;
        this.BLUE_RELATIVE_END_POSE = blueRelativeEndPose;
        this.backOut = backOut;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        this.endPose = FieldConstants.allianceFlip(BLUE_RELATIVE_END_POSE);
        this.startPose = driveBase.getPose();
        PathPlannerTrajectory traj = PathPlanner.generatePath(RobotConstants.kAutoPathConstraints,
                calcStartPoint(endPose), calcEndPoint(startPose));
        driveBase.getPoseEstimator().addTrajectory(traj);

        pathDrivingCommand = driveBase.getFollowTrajectoryCommand(traj);
        MustangScheduler.getInstance().schedule(pathDrivingCommand, driveBase);
    }

    @Override
    public boolean isFinished() {
        return (pathDrivingCommand == null || !pathDrivingCommand.isScheduled());
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            pathDrivingCommand.cancel();
        }
        // driveBase.stop();
        driveBase.getPoseEstimator().removeTrajectory();
    }

    // calcs start point and points directly towards end point
    private PathPoint calcStartPoint(Pose2d nextPose) {
        if (backOut) {
            return new PathPoint(startPose.getTranslation(), startPose.getRotation().rotateBy(new Rotation2d(Math.PI)),
                    startPose.getRotation());
        } else {
            double dx, dy;
            dx = nextPose.getX() - startPose.getX();
            dy = nextPose.getY() - startPose.getY();
            return new PathPoint(startPose.getTranslation(), new Rotation2d(dx, dy),
                    startPose.getRotation());
        }
        // return PathPoint.fromCurrentHolonomicState(startPose, driveBase.getChassisSpeeds()); //
        // turns, doesn't move directly but more accurate?
    }

    // end point where robot faces end Pose
    private PathPoint calcEndPoint(Pose2d prevPose) {
        double dx, dy;
        dx = endPose.getX() - prevPose.getX();
        dy = endPose.getY() - prevPose.getY();
        return new PathPoint(endPose.getTranslation(), new Rotation2d(dx, dy),
                endPose.getRotation().rotateBy(new Rotation2d(Math.PI)));
    }
}
