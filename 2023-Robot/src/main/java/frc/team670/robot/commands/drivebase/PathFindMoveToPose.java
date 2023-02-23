package frc.team670.robot.commands.drivebase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.pathfinder.PoseNode;
import frc.team670.robot.subsystems.pathfinder.ObstacleAvoidanceAStarMap;

public class PathFindMoveToPose extends CommandBase implements MustangCommand {

	private final DriveBase driveBase;
	private MustangPPSwerveControllerCommand pathDrivingCommand;
	private final PoseNode endPoint;
	private PoseNode startPoint;
	private ObstacleAvoidanceAStarMap AStarMap;

	public PathFindMoveToPose(DriveBase driveBase, Pose2d finalPose) {
		this.driveBase = driveBase;
		this.endPoint = new PoseNode(finalPose);
		addRequirements(driveBase);
	}
	
	@Override
	public void initialize() {
		this.startPoint = new PoseNode(driveBase.getPoseEstimator().getCurrentPose());
		this.AStarMap = new ObstacleAvoidanceAStarMap(startPoint, endPoint, FieldConstants.obstacles);
		startPoint = new PoseNode(driveBase.getPoseEstimator().getCurrentPose());

		PathPlannerTrajectory trajectory;
		List<PoseNode> fullPath = AStarMap.findPath();
		if (fullPath == null)
			return;

		// Depending on if internal points are present, make a new array of the other
		// points in the path.
		PathPoint[] fullPathPoints = getPathPointsFromNodes(fullPath);

		trajectory = PathPlanner.generatePath(RobotConstants.kAutoPathConstraints,
				Arrays.asList(fullPathPoints));
		driveBase.getPoseEstimator().addTrajectory(trajectory);
		pathDrivingCommand = driveBase.getFollowTrajectoryCommand(trajectory);
		pathDrivingCommand.schedule();
	}

	private PathPoint[] getPathPointsFromNodes(List<PoseNode> fullPath) {
		PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];

		Rotation2d Heading = new Rotation2d(fullPath.get(1).getX() - startPoint.getX(),
				fullPath.get(1).getY() - startPoint.getY());
		double totalDis = 0;
		for (int i = 0; i < fullPath.size() - 1; i++) {
			totalDis += Math.hypot(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
					fullPath.get(i + 1).getY() - fullPath.get(i).getY());
		}

		for (int i = 0; i < fullPath.size(); i++) {
			if (i == 0) { // first node
				fullPathPoints[i] = new PathPoint(
						new Translation2d(startPoint.getX(), startPoint.getY()), Heading,
						driveBase.getPoseEstimator().getCurrentPose().getRotation(),
						Math.hypot(driveBase.getChassisSpeeds().vxMetersPerSecond,
								driveBase.getChassisSpeeds().vyMetersPerSecond));
			} else if (i + 1 == fullPath.size()) { // last node
				fullPathPoints[i] =
						new PathPoint(new Translation2d(endPoint.getX(), endPoint.getY()),
								new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(),
										fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
								endPoint.getHolRot());
			} else {
				fullPathPoints[i] = new PathPoint(
						new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
						new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
								fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
						endPoint.getHolRot());
			}
		}
		return fullPathPoints;
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
	}

	public static double angleAtPercent(double start, double end, double percent) {
		double angleDiff = end - start;
		if (angleDiff > 180) {
			angleDiff -= 360;
		} else if (angleDiff < -180) {
			angleDiff += 360;
		}
		double angle = start + (angleDiff * percent);
		if (angle > 180) {
			angle -= 360;
		} else if (angle < -180) {
			angle += 360;
		}
		return angle;
	}



	@Override
	public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
		// TODO Auto-generated method stub
		return null;
	}
}
