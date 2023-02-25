import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.util.Units;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.pathfinder.Obstacle;
import frc.team670.robot.pathfinder.ObstacleAvoidanceAStarMap;
import frc.team670.robot.pathfinder.PoseEdge;
import frc.team670.robot.pathfinder.PoseNode;

class ObstacleAvoidanceTest {
    ObstacleAvoidanceAStarMap astar;
    List<PoseNode> trajectory;
    List<PoseEdge> graph;

    @BeforeEach
    void setup() {
       
        astar = new ObstacleAvoidanceAStarMap(
                new PoseNode(Units.inchesToMeters(220), Units.inchesToMeters(100)),
                new PoseNode(Units.inchesToMeters(600), Units.inchesToMeters(100)),
                FieldConstants.obstacles, FieldConstants.obstacleContingencyNodes);
        assert astar != null;
        checkObstacles();
        checkContingencyNode();
    }

    void checkObstacles() {
        System.out.println("\t\t-----OBSTACLE CHECK-----\n");
        for (Obstacle o : FieldConstants.obstacles) {
            double[] xPoints = o.getXPoints();
            double[] yPoints = o.getYPoints();
            
            for (int i = 0; i < xPoints.length; i++) {
                System.out.println(String.format("%f, %f", Units.metersToInches(xPoints[i]), Units.metersToInches(yPoints[i])));
            }
        }
        System.out.println("\t\t-----OBSTACLE CHECK DONE-----\n");
    }

    void checkContingencyNode() {
        System.out.println("\t\t-----CONTINGENCY NODES CHECK-----\n");
        for (PoseNode p : FieldConstants.obstacleContingencyNodes) {
            System.out.println(String.format("%f, %f", p.getX(), p.getY()));
        }
        System.out.println("\t\t-----CONTINGENCY NODES DONE-----\n");
    }

    @Test
    void pathFind() {
        trajectory = astar.findPath();
        graph = astar.getEdges();
        for (PoseEdge e : graph) {
            System.out.println(String.format("%f, %f", Units.metersToInches(e.start.getX()), Units.metersToInches(e.start.getY())));
            System.out.println(String.format("%f, %f", Units.metersToInches(e.end.getX()), Units.metersToInches(e.end.getY())));
            System.out.println();
        }
    }

    @AfterEach
    void getPath() {
        System.out.println("path:\n");
        for (PoseNode node : trajectory) {
            System.out.println(String.format("%f, %f", Units.metersToInches(node.getX()),
                    Units.metersToInches(node.getY())));
        }
    }
}
