package frc.team670.robot.subsystems.pathfinder;

import java.awt.geom.Line2D;
import java.util.List;
import frc.team670.mustanglib.utils.math.sort.AStarMap;
import frc.team670.robot.subsystems.pathfinder.Obstacle.PolygonDouble;

/*
 * Credits: Hemlock 5712
 */

public class ObstacleAvoidanceAStarMap extends AStarMap {

    // Add an edge to the navigation mesh
    public boolean addObstacleToEdge(PoseEdge edge, List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles) {
            PolygonDouble polygon = obstacle.polygon;
            for (int i = 0; i < polygon.npoints; i++) {
                int j = (i + 1) % polygon.npoints;
                double x1 = polygon.xpoints[i];
                double y1 = polygon.ypoints[i];
                double x2 = polygon.xpoints[j];
                double y2 = polygon.ypoints[j];
                if (Line2D.linesIntersect(x1, y1, x2, y2, edge.start.getX(), edge.start.getY(),
                        edge.end.getX(), edge.end.getY())) {
                    return false;
                }
            }
        }
        this.edges.add(edge);
        edge.start.addNeighbor(edge.end);
        edge.end.addNeighbor(edge.start);
        return true;
    }
}
