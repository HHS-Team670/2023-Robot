package frc.team670.robot.commands.drivebase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.pathfinder.Obstacle;
import frc.team670.robot.subsystems.pathfinder.PoseEdge;
import frc.team670.robot.subsystems.pathfinder.PoseNode;
import frc.team670.robot.subsystems.pathfinder.ObstacleAvoidanceAStarMap;

public class PathFindMoveToPose extends CommandBase implements MustangCommand {

	private final DriveBase driveSystem;
	private MustangPPSwerveControllerCommand pathDrivingCommand;
	private final PathConstraints constraints;
	private final PoseNode finalPosition;
	private PoseNode startPoint;
	private ObstacleAvoidanceAStarMap AStarMap;

	public PathFindMoveToPose(DriveBase driveSystem, PathConstraints constraints,
			PoseNode finalPosition, ObstacleAvoidanceAStarMap AStarMap) {
		this.driveSystem = driveSystem;
		this.constraints = constraints;
		this.finalPosition = finalPosition;
		this.AStarMap = AStarMap;
		this.startPoint = new PoseNode(driveSystem.getPoseEstimator().getCurrentPose());

		addRequirements(driveSystem);
	}

	@Override
	public void initialize() {
		startPoint = new PoseNode(driveSystem.getPoseEstimator().getCurrentPose());
		PathPlannerTrajectory trajectory;
		List<PoseNode> fullPath = AStarMap.findPath();
		if (fullPath == null) {
			return;
		}


		// Depending on if internal points are present, make a new array of the other
		// points in the path.
		PathPoint[] fullPathPoints = getPathPointsFromNodes(fullPath);

		trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
		driveSystem.getPoseEstimator().addTrajectory(trajectory);
		pathDrivingCommand = driveSystem.getFollowTrajectoryCommand(trajectory);
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
			if (i == 0) {

				fullPathPoints[i] = new PathPoint(
						new Translation2d(startPoint.getX(), startPoint.getY()), Heading,
						driveSystem.getPoseEstimator().getCurrentPose().getRotation(),
						Math.hypot(driveSystem.getChassisSpeeds().vxMetersPerSecond,
								driveSystem.getChassisSpeeds().vyMetersPerSecond));
			} else if (i + 1 == fullPath.size()) {
				fullPathPoints[i] =
						new PathPoint(new Translation2d(finalPosition.getX(), finalPosition.getY()),
								new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(),
										fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
								finalPosition.getHolRot());
			} else {

				fullPathPoints[i] = new PathPoint(
						new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
						new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
								fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
						finalPosition.getHolRot());
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

		driveSystem.stop();
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