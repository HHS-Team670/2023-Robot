import java.util.List;
import org.junit.jupiter.api.AfterAll;
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
                new PoseNode(Units.inchesToMeters(46.25), Units.inchesToMeters(86.19)),
                FieldConstants.obstacles);

        astar.addNode(new PoseNode(2.92 - 0.42, 1.51 - 0.42));
        astar.addNode(new PoseNode(2.92 - 0.42, 3.98 + 0.42));
        astar.addNode(new PoseNode(4.86 + 0.42, 3.98 + 0.42));
        astar.addNode(new PoseNode(4.86 + 0.42, 1.51 - 0.42));

        // for (Obstacle o : FieldConstants.obstacles) {
        //     // List<Pair<Double, Double>> points = new ArrayList<>();
        //     for (int i = 0; i < o.getXPoints().length; i++) {
        //         double x = o.getXPoints()[i];
        //         double y = o.getYPoints()[i];
        //         System.out.println(String.format("(%f, %f)", Units.metersToInches(x), Units.metersToInches(y)));
        //         // points.add(new Pair<Double,Double>(x, y));
        //     }
        //     System.out.println();
        // }

        // astar.addNode(new PoseNode(11.68 - 0.42, 1.51 - 0.42));
        // astar.addNode(new PoseNode(11.65, 5));
        // astar.addNode(new PoseNode(14.37, 4.55));
        // astar.addNode(new PoseNode(14.23, 1.51 - 0.42));
    }

    @Test
    void pathFind() {
        trajectory = astar.findPath();
        graph = astar.getEdges();
        for (PoseEdge e : graph) {
            System.out.println(String.format("%f, %f", e.start.getX(), e.start.getY()));
            System.out.println(String.format("%f, %f", e.end.getX(), e.end.getY()));
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
