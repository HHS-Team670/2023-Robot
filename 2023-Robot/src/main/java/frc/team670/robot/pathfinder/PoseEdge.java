package frc.team670.robot.pathfinder;

import frc.team670.mustanglib.utils.math.sort.Edge;

public class PoseEdge implements Edge<PoseNode> {
    public PoseNode start, end;
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
    public PoseNode getSource() {
        return start;
    }

    @Override
    public PoseNode getDest() {
        return end;
    }
}
