package frc.team670.robot.subsystems.pathfinder;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team670.mustanglib.utils.math.sort.Node;
import frc.team670.robot.subsystems.PoseEstimatorSubsystem;

/*
 * Credits: Hemlock 5712
 */

public class PoseNode implements Node<PoseNode> {
	private double x, y;
	private Rotation2d holonomicRotation;
	private List<PoseNode> neighbors;

	public PoseNode(double x, double y) {
		this.x = x;
		this.y = y;
		holonomicRotation = Rotation2d.fromDegrees(0);
		this.neighbors = new ArrayList<>();
	}

	public PoseNode(double x, double y, Rotation2d holonomicRotation) {
		this.x = x;
		this.y = y;
		this.holonomicRotation = holonomicRotation;
		this.neighbors = new ArrayList<>();
	}

	public PoseNode(PoseEstimatorSubsystem p) {
		this.x = p.getCurrentPose().getX();
		this.y = p.getCurrentPose().getY();
		this.holonomicRotation = p.getCurrentPose().getRotation();
		this.neighbors = new ArrayList<>();
	}

	public PoseNode(Translation2d coordinates, Rotation2d holonomicRotation) {
		this.x = coordinates.getX();
		this.y = coordinates.getY();
		this.holonomicRotation = holonomicRotation;
		this.neighbors = new ArrayList<>();
	}

	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	public Rotation2d getHolRot() {
		return holonomicRotation;
	}

	public void setHolRot(double degree) {
		this.holonomicRotation = Rotation2d.fromDegrees(degree);
	}

	@Override
	public void addNeighbor(PoseNode neighbor) {
		this.neighbors.add((PoseNode) neighbor);
	}
	
	@Override
	public double getHeuristicDistance(PoseNode target) {
		PoseNode n1 = this;
		PoseNode n2 = (PoseNode) target;

		double dx = n1.x - n2.x;
        double dy = n1.y - n2.y;
        return Math.hypot(dx, dy);
	}

	@Override
	public List<PoseNode> getNeighbors() {
		return neighbors;
	}
}
