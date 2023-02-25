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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.mustanglib.utils.MustangController;

/**
 * MoveToPose - moves to specified pose. Cancels when button is released.
 */
public class MoveToPose extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    private Pose2d goalPose;
    protected Map<MustangSubsystemBase, HealthState> healthReqs;

    private MustangPPSwerveControllerCommand pathDrivingCommand;

    public MoveToPose(DriveBase driveBase, Pose2d goalPose) {
        this.driveBase = driveBase;
        this.goalPose = goalPose;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
    }

    // public MoveToPose(DriveBase driveBase, Pose2d goalPose) {
    //     this.driveBase = driveBase;
    //     path = null;
    //     this.goalPose = goalPose;
    //     this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    //     this.healthReqs.put(driveBase, HealthState.GREEN);
    //     addRequirements(driveBase);
    // }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        goalPose = FieldConstants.allianceFlip(goalPose).transformBy(new Transform2d(FieldConstants.allianceFlip(new Translation2d(RobotConstants.DRIVEBASE_WIDTH + 0.1, 0)), FieldConstants.getRobotFacingRotation()));
        SmartDashboard.putString("GOAL POSE", String.format("%f, %f, %f degrees", goalPose.getX(), goalPose.getY(), goalPose.getRotation().getDegrees()));
        PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(1, 0.5), calcStartPoint(),
                calcEndPoint(goalPose));
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
            SmartDashboard.putBoolean("LET GO", interrupted);
			pathDrivingCommand.cancel();
		}

		driveBase.stop();
        // driveBase.getPoseEstimator().removeTrajectory();
    }


    private PathPoint calcStartPoint() {
        return new PathPoint(driveBase.getPoseEstimator().getCurrentPose().getTranslation(), new Rotation2d(),
                driveBase.getGyroscopeRotation());
    }

    private PathPoint calcEndPoint(Pose2d targetPose) {
        return new PathPoint(targetPose.getTranslation(), new Rotation2d(),
                driveBase.getGyroscopeRotation());
    }
}
