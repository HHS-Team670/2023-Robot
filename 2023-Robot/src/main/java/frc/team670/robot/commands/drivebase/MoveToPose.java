package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.commands.vision.IsLockedOn;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * MoveToPose - moves to specified pose. Cancels when button is released.
 */
public class MoveToPose extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    private final Pose2d BLUE_RELATIVE_END_POSE;
    Pose2d endPose;
    Pose2d startPose;
    protected Map<MustangSubsystemBase, HealthState> healthReqs;

    private MustangPPSwerveControllerCommand pathDrivingCommand;

    public MoveToPose(DriveBase driveBase, Pose2d blueRelativeEndPose) {
        this.driveBase = driveBase;
        this.BLUE_RELATIVE_END_POSE = blueRelativeEndPose;
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
        SmartDashboard.putString("MOVETOPOSE: END POSE", String.format("(%.2f, %.2f)", endPose.getX(), endPose.getY()));
        this.startPose = driveBase.getPoseEstimator().getCurrentPose();
        PathPlannerTrajectory traj = PathPlanner.generatePath(RobotConstants.kAutoPathConstraints,
                calcStartPoint(endPose), calcEndPoint(startPose));
        driveBase.getPoseEstimator().addTrajectory(traj);

        pathDrivingCommand = driveBase.getFollowTrajectoryCommand(traj);
        MustangScheduler.getInstance().schedule(pathDrivingCommand, driveBase);
        MustangScheduler.getInstance().schedule(new IsLockedOn(driveBase, endPose));
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
        driveBase.stop();
        driveBase.getPoseEstimator().removeTrajectory();
    }

    // calcs start point and points directly towards end point
    private PathPoint calcStartPoint(Pose2d nextPose) {
        double dx, dy;
        dx = nextPose.getX() - startPose.getX();
        dy = nextPose.getY() - startPose.getY();
        return new PathPoint(startPose.getTranslation(), new Rotation2d(dx, dy), startPose.getRotation()); 
        // return PathPoint.fromCurrentHolonomicState(startPose, driveBase.getChassisSpeeds());    // TODO: TEST
    }

    // end point where robot faces end Pose
    private PathPoint calcEndPoint(Pose2d prevPose) {
        double dx, dy;
        dx = endPose.getX() - prevPose.getX();
        dy = endPose.getY() - prevPose.getY();
        return new PathPoint(endPose.getTranslation(), new Rotation2d(dx, dy), endPose.getRotation());
    }
}
