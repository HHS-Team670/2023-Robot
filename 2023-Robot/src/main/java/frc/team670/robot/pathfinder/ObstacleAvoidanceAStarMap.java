package frc.team670.robot.pathfinder;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;
import com.fasterxml.jackson.databind.ext.OptionalHandlerFactory;
import frc.team670.mustanglib.utils.math.sort.AStarSearch;
import frc.team670.robot.pathfinder.Obstacle.PolygonDouble;

/**
 * @author ethan c :D
 */
public class ObstacleAvoidanceAStarMap {

    private final AStarSearch<PoseNode, PoseEdge> searchAlg = new AStarSearch<>();
    private List<PoseNode> contingencyNodes = new ArrayList<>();
    private final List<PoseEdge> edges = new ArrayList<>();
    private final List<Obstacle> obstacles = new ArrayList<>();
    private PoseNode startNode, endNode;

    public ObstacleAvoidanceAStarMap(PoseNode start, PoseNode destination, List<Obstacle> obstacles) {
        this.startNode = start;
        this.endNode = destination;
        addObstacles(obstacles);
    }

    public ObstacleAvoidanceAStarMap(PoseNode start, PoseNode destination, List<Obstacle> obstacles, List<PoseNode> obstacleContingencyNodes) {
        this.startNode = start;
        this.endNode = destination;
        this.contingencyNodes = obstacleContingencyNodes;
        addObstacles(obstacles);
    }

    // Add a node to the navigation mesh
    public void addNode(PoseNode node) {
        this.contingencyNodes.add(node);
    }

    public void addNodes(List<PoseNode> nodes) {
        this.contingencyNodes.addAll(nodes);
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
            System.out.println("intersects obstacle!");
            loadMap();
            fullPath = searchAlg.search(startNode, endNode);
        } else {
            System.out.println("no intersection");
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
        PoseEdge startToContingency;
        for (PoseNode node : contingencyNodes) {
            startToContingency = new PoseEdge(startNode, node);
            if (!intersectsObstacles(startToContingency)) {
                this.edges.add(startToContingency);
                startToContingency.start.addNeighbor(startToContingency.end);
                startToContingency.end.addNeighbor(startToContingency.start);
            }
        }
        PoseEdge contingencyToEnd;
        for (PoseNode node : contingencyNodes) {
            contingencyToEnd = new PoseEdge(node, endNode);
            if (!intersectsObstacles(contingencyToEnd)) {
                this.edges.add(contingencyToEnd);
                contingencyToEnd.start.addNeighbor(contingencyToEnd.end);
                contingencyToEnd.end.addNeighbor(contingencyToEnd.start);
            }
        }
        PoseEdge contTocont;
        for (PoseNode node : contingencyNodes) {
            for (PoseNode other : contingencyNodes) {
                if (node == other) continue;

                contTocont = new PoseEdge(node, other);
                if(!intersectsObstacles(contTocont)) {
                    this.edges.add(contTocont);
                    contTocont.start.addNeighbor(contTocont.end);
                    contTocont.end.addNeighbor(contTocont.start);
                }
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
                    return true; // if line from start to final interesects with any obstacle lines
                }
            }
        }
        return false;
    }

    public List<PoseEdge> getEdges() {
        return edges;
    }
}
