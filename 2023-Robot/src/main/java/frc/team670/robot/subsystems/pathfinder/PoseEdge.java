package frc.team670.robot.subsystems.pathfinder;

import frc.team670.mustanglib.utils.math.sort.Edge;
import frc.team670.mustanglib.utils.math.sort.Node;

public class PoseEdge implements Edge {
    PoseNode start, end;
    private double cost;

    public PoseEdge(PoseNode start, PoseNode end, double cost) {
        this.start = start;
        this.end = end;
        this.cost = cost;
    }

    public PoseEdge(PoseNode start, PoseNode end) {
        this.start = start;
        this.end = end;
        this.cost = start.getHeuristicDistance(end);
    }

    @Override
    public double getCost() {
        return cost;
    }

    @Override
    public Node getSource() {
        return start;
    }

    @Override
    public Node getDest() {
        return end;
    }
}
