package frc.team670.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

/**
 * Represents the whole Arm system, containing multiple joints.
 * Models the arm as a state machine.
 * 
 * @author Armaan, Aditi, Alexander, Gabriel, Kedar, Justin, Sanatan, Srinish
 */
public class Arm extends MustangSubsystemBase {
    private Shoulder shoulder;
    private Elbow elbow;
    private ArmState targetState;
    private boolean initializedState;

    private ArmSegment shoulderSegment, elbowSegment, claw;

    private static final ArmState[][] VALID_PATHS_GRAPH = new ArmState[][] {
            { ArmState.TUNING, ArmState.SCORE_MID, ArmState.INTERMEDIATE_BACKWARD_GROUND }, // STOWED
            { ArmState.SCORE_MID}, // HYBRID
            { ArmState.STOWED, ArmState.SCORE_HIGH, ArmState.HYBRID}, // SCORE_MID
            { ArmState.SCORE_MID }, // SCORE_HIGH
            { ArmState.BACKWARD_GROUND, ArmState.STOWED}, // INTERMEDIATE_BACKWARD_GROUND
            { ArmState.INTERMEDIATE_BACKWARD_GROUND }, // BACKWARD_GROUND
            { ArmState.STOWED }, // TUNING

    };

    private static ArmState VALID_PATHS[][][] = new ArmState[VALID_PATHS_GRAPH.length][VALID_PATHS_GRAPH.length][];

    public Arm() {
        this.shoulder = new Shoulder();
        this.elbow = new Elbow();
        this.targetState = ArmState.STOWED; // TODO: Make this be the closest position (getClosestState)
        this.initializedState = false;

        init();
    }

    private static void init() {
        for (int i = 0; i < VALID_PATHS_GRAPH.length; i++) {
            for (int j = 0; j < VALID_PATHS_GRAPH.length; j++) {
                // set validpaths[i][j] to the path between i and j
                VALID_PATHS[i][j] = getValidPath(ArmState.getVal(i), ArmState.getVal(j));
            }
        }
    }

    @Override
    public HealthState checkHealth() {
        if (elbow.checkHealth() == HealthState.RED) {
            return HealthState.RED;
        }

        if (elbow.checkHealth() == HealthState.RED || shoulder.checkHealth() == HealthState.RED) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        debugSubsystem();
        if (!initializedState) {
            if (elbow.isRelativePositionSet() && shoulder.isRelativePositionSet()) {
                initializedState = true;
                this.targetState = getClosestState();
            }
        }
    }

    public void resetPositionFromAbsolute() {
        initializedState = false;
        elbow.resetPositionFromAbsolute();
        shoulder.resetPositionFromAbsolute();
    }

    /**
     * This moves DIRECTLY to the target ArmState
     * We must handle checking for valid paths ELSEWHERE.
     */
    public void moveToTarget(ArmState target) {
        this.targetState = target;
        elbow.setEncoderPositionFromAbsolute();
        shoulder.setEncoderPositionFromAbsolute();
        elbow.setSystemTargetAngleInDegrees(target.getElbowAngle());
        shoulder.setSystemTargetAngleInDegrees(target.getShoulderAngle());
        // elbow.updateSoftLimits(new float[] {,});
    }

    public void updateArbitraryFeedForwards() {
        elbow.updateArbitraryFeedForward(shoulder.getCurrentAngleInDegrees());
        shoulder.updateArbitraryFeedForward(elbow.getCurrentAngleInDegrees());
    }

    /**
     * Returns the state we're moving towards
     * Ex: If we're moving from A to B, this returns B
     */
    public ArmState getTargetState() {

        return targetState;
    }

    /**
     * Returns whether or not the arm is physically at the targetState
     * 
     * 
     */
    public boolean hasReachedTargetPosition() {
        return shoulder.hasReachedTargetPosition() && elbow.hasReachedTargetPosition();
    }

    /**
     * Returns a valid list of states from the starting State to the ending State
     * This path should include BOTH the starting state AND the ending state
     */
    public static ArmState[] getValidPath(ArmState start, ArmState finish) {
        // retrieve the list of intermediate states from VALID_PATHS
        // if Validpath[start][finish]==null then run dykestras below else return
        // validpaths[start[finish]]
        // System.out.println(Arrays.toString(VALID_PATHS[start.getStateID()][finish.getStateID()]));
        if (VALID_PATHS[start.getStateID()][finish.getStateID()] == null) {

            ArrayList<ArmState> tempValidPath = null;
            PriorityQueue<Pair> queue = new PriorityQueue<Pair>();
            ArrayList<ArmState> list = new ArrayList<ArmState>();

            queue.add(new Pair(list, start));
            while (!queue.isEmpty()) {
                Pair v = queue.poll();
                v.path.add(v.node);
                if (v.node == finish) {
                    tempValidPath = v.path;
                    break;
                }
                for (ArmState state : VALID_PATHS_GRAPH[v.node.getStateID()]) {
                    list = new ArrayList<>(v.path);
                    queue.add(new Pair(list, state));
                }

            }
            if (tempValidPath != null) {
                ArmState[] path = tempValidPath.toArray(new ArmState[tempValidPath.size()]);
                if (path.length == 0) {
                    System.out.println("No valid path found between " + start + " and " + finish);
                }
                return path;
            }

            return new ArmState[] {};

        } else {
            return VALID_PATHS[start.getStateID()][finish.getStateID()];
        }

    }

    @Override
    public void debugSubsystem() {
        shoulder.debugSubsystem();
        elbow.debugSubsystem();
        SmartDashboard.putString("Arm target state", getTargetState().toString());
    }

    static class Pair implements Comparable<Pair> {
        ArmState node;
        ArrayList<ArmState> path;

        public Pair(ArrayList<ArmState> path, ArmState node) {
            this.node = node;
            this.path = path;
        }

        @Override
        public int compareTo(Pair o) {
            if (this.path.size() < o.path.size())
                return 0;
            else if (this.path.size() > o.path.size())
                return 1;
            else if (this.node.getStateID() < o.node.getStateID())
                return 0;
            else
                return 1;

        }
    }

    // prioritize elbow accuracy
    public ArmState getClosestState() {
        double shoulderAngle = shoulder.getCurrentAngleInDegrees();
        double elbowAngle = elbow.getCurrentAngleInDegrees();
        ArmState closestState = ArmState.STOWED;
        double closestStateDistance = 10000; //high number so first one will be less

        for (ArmState state : ArmState.values()) {
            double shoulderDistance = Math.abs(state.getShoulderAngle() - shoulderAngle);
            double elbowDistance = Math.abs(state.getElbowAngle() - elbowAngle);
            double stateDistance = shoulderDistance * 1.5 + elbowDistance;
            if(stateDistance < closestStateDistance) {
                closestStateDistance = stateDistance;
                closestState = state;
            }
        }
        return closestState;
    }

    public Shoulder getShoulder() {
        return shoulder;
    }

    public Elbow getElbow() {
        return elbow;
    }
}
