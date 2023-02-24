package frc.team670.robot.pathfinder;

import java.util.List;

import edu.wpi.first.math.util.Units;
import frc.team670.robot.constants.FieldConstants;

public class test {
    public static void main(String[] args) {
        ObstacleAvoidanceAStarMap astar = new ObstacleAvoidanceAStarMap(
                new PoseNode(Units.inchesToMeters(150), Units.inchesToMeters(100)), new PoseNode(Units.inchesToMeters(46.25), Units.inchesToMeters(86.19)),
                FieldConstants.obstacles);
        List<PoseNode> traj = astar.findPath();
        for (PoseNode node : traj) {
            System.out.println(String.format("(%f, %f)", node.getX(), node.getY()));
        }
    }
}
