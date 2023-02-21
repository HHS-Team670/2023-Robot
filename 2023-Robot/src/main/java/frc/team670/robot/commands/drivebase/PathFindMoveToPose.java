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
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.robot.subsystems.PoseEstimatorSubsystem;
import frc.team670.robot.subsystems.pathfinder.Obstacle;
import frc.team670.robot.subsystems.pathfinder.PoseEdge;
import frc.team670.robot.subsystems.pathfinder.PoseNode;
import frc.team670.robot.subsystems.pathfinder.ObstacleAvoidanceAStarMap;

public class PathFindMoveToPose extends CommandBase implements MustangCommand {

	private final DriveBase driveSystem;
	private MustangPPSwerveControllerCommand pathDrivingCommand;
	private final PoseEstimatorSubsystem poseEstimatorSubsystem;
	private final PathConstraints constraints;
	private final PoseNode finalPosition;
	private PoseNode startPoint;
	private final List<Obstacle> obstacles;
	private ObstacleAvoidanceAStarMap AStarMap;

	public PathFindMoveToPose(DriveBase driveSystem, PoseEstimatorSubsystem p,
			PathConstraints constraints, PoseNode finalPosition, List<Obstacle> obstacles,
			ObstacleAvoidanceAStarMap AStarMap) {
		this.driveSystem = driveSystem;
		this.poseEstimatorSubsystem = p;
		this.constraints = constraints;
		this.obstacles = obstacles;
		this.finalPosition = finalPosition;
		this.AStarMap = AStarMap;
		this.startPoint = new PoseNode(p);

		addRequirements(driveSystem);
	}

	@Override
	public void initialize() {
		startPoint = new PoseNode(poseEstimatorSubsystem);
		PathPlannerTrajectory trajectory;
		List<PoseNode> fullPath = new ArrayList<PoseNode>();

		AStarMap.addNode(startPoint);
		if (AStarMap.addObstacleToEdge(new PoseEdge(startPoint, finalPosition), obstacles)) {
			fullPath.add(0, startPoint);
			fullPath.add(1, finalPosition);
		} else {
			for (int i = 0; i < AStarMap.getNodeSize(); i++) {
				PoseNode endNode = AStarMap.getNode(i);
				AStarMap.addObstacleToEdge(new PoseEdge(startPoint, endNode), obstacles);
			}
			fullPath = AStarMap.findPath(startPoint, finalPosition);
		}

		if (fullPath == null) {
			return;
		}
		Rotation2d Heading = new Rotation2d(fullPath.get(1).getX() - startPoint.getX(),
				fullPath.get(1).getY() - startPoint.getY());
		double totalDis = 0;
		for (int i = 0; i < fullPath.size() - 1; i++) {
			totalDis += Math.hypot(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
					fullPath.get(i + 1).getY() - fullPath.get(i).getY());
		}

		// Depending on if internal points are present, make a new array of the other
		// points in the path.
		PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];

		for (int i = 0; i < fullPath.size(); i++) {
			if (i == 0) {

				fullPathPoints[i] =
						new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()),
								Heading, poseEstimatorSubsystem.getCurrentPose().getRotation(),
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
				// fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(),
				// fullPath.get(i).getY()),
				// new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
				// fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
				// (Rotation2d)null);
			}

		}

		// Declare an array to hold PathPoint objects made from all other points
		// specified in constructor.
		trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
		poseEstimatorSubsystem.addTrajectory(trajectory);
		pathDrivingCommand =
				driveSystem.getFollowTrajectoryCommand(trajectory, poseEstimatorSubsystem);
		pathDrivingCommand.schedule();
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
