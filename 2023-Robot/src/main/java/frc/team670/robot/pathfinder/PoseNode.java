package frc.team670.robot.pathfinder;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team670.mustanglib.utils.math.sort.Node;

/*
 * Credits: Hemlock 5712
 */

public class PoseNode implements Node<PoseNode> {
	private Pose2d pose;
	private List<PoseNode> neighbors;

	public PoseNode(double x, double y) {
		this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(0));
		this.neighbors = new ArrayList<>();
	}

	public PoseNode(double x, double y, Rotation2d holonomicRotation) {
		this.pose = new Pose2d(x, y, Rotation2d.fromDegrees(0));
		this.neighbors = new ArrayList<>();
	}

	public PoseNode(Pose2d currentPose) {
		this.pose = currentPose;
		this.neighbors = new ArrayList<>();
	}

	public PoseNode(Translation2d coordinates, Rotation2d holonomicRotation) {
		this.pose = new Pose2d(coordinates, holonomicRotation);
		this.neighbors = new ArrayList<>();
	}


	public double getX() {
		return pose.getX();
	}

	public double getY() {
		return pose.getY();
	}

	public Rotation2d getHolRot() {
		return pose.getRotation();
	}

	@Override
	public void addNeighbor(PoseNode neighbor) {
		this.neighbors.add(neighbor);
	}

	@Override
	public double getHeuristicDistance(PoseNode target) {
		return this.pose.getTranslation().getDistance(target.pose.getTranslation());
	}

	@Override
	public List<PoseNode> getNeighbors() {
		return neighbors;
	}
}
