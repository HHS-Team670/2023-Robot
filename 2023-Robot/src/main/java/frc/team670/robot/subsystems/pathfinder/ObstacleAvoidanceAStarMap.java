package frc.team670.robot.subsystems.pathfinder;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;
import frc.team670.mustanglib.utils.math.sort.AStarSearch;
import frc.team670.robot.subsystems.pathfinder.Obstacle.PolygonDouble;

/*
 * Credits: Hemlock 5712
 */

public class ObstacleAvoidanceAStarMap {

    private final AStarSearch<PoseNode, PoseEdge> searchAlg = new AStarSearch<>();
    private final List<PoseNode> contingencyNodes = new ArrayList<>();
    private final List<PoseEdge> edges = new ArrayList<>();
    private final List<Obstacle> obstacles = new ArrayList<>();
    private PoseNode startNode, endNode;

    public ObstacleAvoidanceAStarMap(PoseNode start, PoseNode destination, List<Obstacle> obstacles) {
        this.startNode = start;
        this.endNode = destination;
        addObstacles(obstacles);
    }

    // Add a node to the navigation mesh
    public void addNode(PoseNode node) {
        this.contingencyNodes.add(node);
    }

    public int getNodeSize() {
        return contingencyNodes.size();
    }

    public PoseNode getNode(int index) {
        return contingencyNodes.get(index);
    }

    public List<PoseNode> findPath() {
        List<PoseNode> fullPath = new ArrayList<>();
        if (intersectsObstacles(new PoseEdge(startNode, endNode))) {
            loadMap();
            fullPath = searchAlg.search(startNode, endNode);
        } else {
            fullPath.add(startNode);
            fullPath.add(endNode);
        }
        return fullPath;
    }

    public void addObstacles(List<Obstacle> obstacles) {
        this.obstacles.addAll(obstacles);
    }

    // Add edges to nodes it doesn't intersect obstacles
    private void loadMap() {
        PoseEdge startToContingencyNode;
        int firstHalf = 0;
        for (PoseNode node : contingencyNodes) {
            startToContingencyNode = new PoseEdge(startNode, node);
            if (!intersectsObstacles(startToContingencyNode)) {
                this.edges.add(startToContingencyNode);
                startToContingencyNode.start.addNeighbor(startToContingencyNode.end);
                startToContingencyNode.end.addNeighbor(startToContingencyNode.start);
                firstHalf++;
            }
        }
        PoseEdge contingencyToEndNode;
        for (int i = 0; i < firstHalf; i++) {
            contingencyToEndNode = new PoseEdge(edges.get(i).end, endNode);
            if (!intersectsObstacles(contingencyToEndNode)) {
                this.edges.add(contingencyToEndNode);
                contingencyToEndNode.start.addNeighbor(contingencyToEndNode.end);
                contingencyToEndNode.end.addNeighbor(contingencyToEndNode.start);
            }
        }
    }

    private boolean intersectsObstacles(PoseEdge edge) {
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
                    return false; // if line from start to final interesects with any obstacle lines
                }
            }
        }
        return true;
    }
}
