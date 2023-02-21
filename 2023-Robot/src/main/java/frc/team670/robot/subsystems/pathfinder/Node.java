package frc.team670.robot.subsystems.pathfinder;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.robot.subsystems.PoseEstimatorSubsystem;

/*
 * Credits: Hemlock 5712
 */

public class Node {
  double x, y;
  Rotation2d holonomicRotation;
  List<Node> neighbors;

  public Node(double x, double y) {
    this.x = x;
    this.y = y;
    holonomicRotation = Rotation2d.fromDegrees(0);
    this.neighbors = new ArrayList<>();
  }

  public Node(double x, double y, Rotation2d holonomicRotation) {
    this.x = x;
    this.y = y;
    this.holonomicRotation = holonomicRotation;
    this.neighbors = new ArrayList<>();
  }

  public Node(PoseEstimatorSubsystem p) {
    this.x = p.getCurrentPose().getX();
    this.y = p.getCurrentPose().getY();
    this.holonomicRotation = p.getCurrentPose().getRotation();
    this.neighbors = new ArrayList<>();
  }

  public Node(Translation2d coordinates, Rotation2d holonomicRotation) {
    this.x = coordinates.getX();
    this.y = coordinates.getY();
    this.holonomicRotation = holonomicRotation;
    this.neighbors = new ArrayList<>();
  }

  public void addNeighbor(Node neighbor) {
    this.neighbors.add(neighbor);
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
}
