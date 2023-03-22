package frc.team670.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import static java.util.Map.entry;
import java.util.PriorityQueue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.constants.RobotConstants; 

/**
 * Represents the whole Arm system, containing multiple joints.
 * Models the arm as a state machine.
 * 
 * @author Armaan, Aditi, Alexander, Gabriel, Kedar, Justin, Sanatan, Srinish
 */
public class Arm extends MustangSubsystemBase {
    private Shoulder shoulder;
    private Elbow elbow;
    private Wrist wrist;
    private ArmState targetState;
    private boolean initializedState;
    private VoltageCalculator voltageCalculator;

    private double elbowOffset;
    private double shoulderOffset;
    private double wristOffset;

    private boolean hasSetShoulderTarget = true;
    private boolean hasSetElbowTarget = true;
    private boolean hasSetWristTarget = true;

    private final String targetPositionKey = "Arm target position";
    
    private long startingTime = 0;

    private double[] currentTimeDelays = new double[] {0, 0, 0};


    private static final ArmState[][] VALID_PATHS_GRAPH = new ArmState[][] {
            { ArmState.TUNING, ArmState.SCORE_MID, ArmState.SINGLE_STATION, ArmState.SCORE_HIGH, ArmState.HYBRID, ArmState.INTAKE_SHELF, ArmState.UPRIGHT_GROUND }, // STOWED
            { ArmState.STOWED, ArmState.SCORE_MID, ArmState.SCORE_HIGH, ArmState.INTAKE_SHELF, ArmState.UPRIGHT_GROUND}, // HYBRID
            { ArmState.SCORE_HIGH, ArmState.STOWED, ArmState.STARTING, ArmState.HYBRID, ArmState.SINGLE_STATION, ArmState.UPRIGHT_GROUND}, // SCORE_MID
            { ArmState.SCORE_MID,  ArmState.STOWED, ArmState.HYBRID, ArmState.SINGLE_STATION, ArmState.INTAKE_SHELF, ArmState.STARTING, ArmState.UPRIGHT_GROUND}, // SCORE_HIGH
            { ArmState.SCORE_MID, ArmState.INTAKE_SHELF, ArmState.SCORE_HIGH}, // STARTING
            { ArmState.STOWED}, // TUNING
            { ArmState.STOWED, ArmState.SCORE_MID, ArmState.SCORE_HIGH, ArmState.INTAKE_SHELF}, // SINGLE_STATION
            { ArmState.SCORE_HIGH, ArmState.STOWED, ArmState.STARTING, ArmState.HYBRID, ArmState.SINGLE_STATION, ArmState.UPRIGHT_GROUND}, // INTAKE_SHELF
            { ArmState.STOWED, ArmState.SCORE_MID, ArmState.SCORE_HIGH, ArmState.INTAKE_SHELF, ArmState.HYBRID}, // UPRIGHT_GROUND

    };

    private static final Map<ArmState, Map<ArmState, double[]>> timeDelays = Map.ofEntries(
        entry(ArmState.STOWED, Map.ofEntries( //From STOWED
                entry(ArmState.SCORE_HIGH, new double[]{400, 0, 250}),
                entry(ArmState.SCORE_MID, new double[]{400, 0, 250}),
                entry(ArmState.HYBRID, new double[]{0, 0, 0})
            )    
        ),
        entry(ArmState.HYBRID, Map.ofEntries( //From HYBRID
                entry(ArmState.STOWED, new double[]{0, 0, 0}),
                entry(ArmState.SCORE_MID, new double[]{0, 150, 400}),
                entry(ArmState.INTAKE_SHELF, new double[]{0, 150, 400})
            )
        ),
        entry(ArmState.SCORE_MID, Map.ofEntries( //From SCORE_MID
                entry(ArmState.STOWED, new double[]{0, 250, 0}),
                entry(ArmState.HYBRID, new double[]{0, 250, 0}),
                entry(ArmState.SCORE_HIGH, new double[]{250, 0, 250})
            )
        ),
        entry(ArmState.SCORE_HIGH, Map.ofEntries( //From SCORE_HIGH
                entry(ArmState.STOWED, new double[]{0, 0, 0}),
                entry(ArmState.HYBRID, new double[]{250, 500, 0})
            )
        ),
        entry(ArmState.STARTING, Map.ofEntries( //From STARTING
                entry(ArmState.SCORE_MID, new double[]{500, 0, 500}),
                entry(ArmState.INTAKE_SHELF, new double[]{500, 0, 500}),
                entry(ArmState.SCORE_HIGH, new double[]{500, 0, 500})
            )
        ),
        entry(ArmState.INTAKE_SHELF, Map.ofEntries( //From SCORE_MID
                entry(ArmState.STOWED, new double[]{0, 250, 0}),
                entry(ArmState.HYBRID, new double[]{0, 250, 0})
            )
        ),
        entry(ArmState.UPRIGHT_GROUND, Map.ofEntries( //From UPRIGHT_GROUND
                entry(ArmState.STOWED, new double[]{0, 0, 0}),
                entry(ArmState.SCORE_MID, new double[]{0, 150, 400}),
                entry(ArmState.INTAKE_SHELF, new double[]{0, 150, 400})
            )
        )
    );

    private static ArmState VALID_PATHS[][][] = new ArmState[VALID_PATHS_GRAPH.length][VALID_PATHS_GRAPH.length][];

    public Arm() {
        this.shoulder = new Shoulder();
        this.elbow = new Elbow();
        this.wrist = new Wrist();
        this.targetState = ArmState.STOWED;
        this.initializedState = false;
        this.voltageCalculator = new VoltageCalculator(RobotConstants.SHOULDER_SEGMENT, RobotConstants.ELBOW_SEGMENT, RobotConstants.WRIST_SEGMENT);

        init();
    }

    /**
     * Private initialization method that populates the grpah of valid paths with every possible state transition
     */
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
        //If one or more joints are RED, then Arm is RED
        if (shoulder.checkHealth() == HealthState.RED || elbow.checkHealth() == HealthState.RED || wrist.checkHealth() == HealthState.RED) {
            return HealthState.RED;
        }

        //If one or more joints are YELLOW, and none are RED, then Arm is YELLOW
        if(shoulder.checkHealth() == HealthState.YELLOW || elbow.checkHealth() == HealthState.YELLOW || wrist.checkHealth() == HealthState.YELLOW) {
            return HealthState.YELLOW;
        }

        return HealthState.GREEN;
    }

    public void setStateToStarting() {
        this.targetState = ArmState.STARTING;
    }

    @Override
    public void mustangPeriodic() {
        if (!initializedState) {
            if (elbow.isRelativePositionSet() && shoulder.isRelativePositionSet() && wrist.isRelativePositionSet()) {
                initializedState = true;
                this.targetState = getClosestState();
            }
        }

        //set target positions for each joint
        double elapsedTime = System.currentTimeMillis() - startingTime;
        
        if(!hasSetShoulderTarget && elapsedTime > currentTimeDelays[0]) {
            hasSetShoulderTarget = true;
            shoulder.setSystemTargetAngleInDegrees(targetState.getShoulderAngle());
        }
        if(!hasSetElbowTarget && elapsedTime > currentTimeDelays[1]) {
            hasSetElbowTarget = true;
            elbow.setSystemTargetAngleInDegrees(targetState.getElbowAngle());
        }
        if(!hasSetWristTarget && elapsedTime > currentTimeDelays[2]) {
            hasSetWristTarget = true;
            wrist.setSystemTargetAngleInDegrees(targetState.getWristAngle());
        }
    }

    /**
     * Public method to reset the arm's state from its absolute position.
     * This will also temporarily set the Arm's HealthState to YELLOW
     * until the relative positions have been properly reset
     */
    public void resetPositionFromAbsolute() {
        // initializedState = false;
        elbow.resetPositionFromAbsolute();
        shoulder.resetPositionFromAbsolute();
        wrist.resetPositionFromAbsolute();
    }

    /**
     * This moves DIRECTLY to the target ArmState
     * We must handle checking for valid paths ELSEWHERE.
     */
    public void moveToTarget(ArmState target) {
        if(checkHealth() == HealthState.GREEN) {
            this.elbowOffset = 0;
            this.shoulderOffset = 0;
            
            hasSetShoulderTarget = false;
            hasSetElbowTarget = false;
            hasSetWristTarget = false;

            currentTimeDelays = null;
            Map<ArmState, double[]> fromStartingState = timeDelays.get(this.targetState);
            if(fromStartingState != null) {
                currentTimeDelays = timeDelays.get(this.targetState).get(target);
                
            }
            if(currentTimeDelays == null) {
                currentTimeDelays = new double[] {0, 0, 0};
            }
            
            SmartDashboard.putString("Arm moveToTarget()", "from " + this.targetState + " to " + target + " is " + Arrays.toString(currentTimeDelays));

            this.targetState = target;

            startingTime = System.currentTimeMillis();
        }
    }

    /**
     * Updates the arbitraryFeedForward of each joint the Arm contains.
     */
    public void updateArbitraryFeedForward() {
        //The -90 here is so that the first joint's position is relative to the GROUND.
        ArrayList<Double> voltages = voltageCalculator.calculateVoltages(shoulder.getCurrentAngleInDegrees() - 90, elbow.getCurrentAngleInDegrees(), wrist.getCurrentAngleInDegrees());
        
        elbow.updateArbitraryFeedForward(voltages.get(0));
        shoulder.updateArbitraryFeedForward(voltages.get(1));
        wrist.updateArbitraryFeedForward(voltages.get(2));
    }

    /**
     * Returns the state we're moving towards
     * Ex: If we're moving from A to B, this returns B
     */
    public ArmState getTargetState() {
        return targetState;
    }

    public void setArmOffsets(double elbowOffset, double shoulderOffset) {
        elbow.setOffset(elbowOffset);
        shoulder.setOffset(shoulderOffset);
        elbow.setSystemTargetAngleInDegrees(targetState.getElbowAngle() + elbow.getOffset());
        shoulder.setSystemTargetAngleInDegrees(targetState.getShoulderAngle() + shoulder.getOffset());

    }

    /**
     * Returns whether or not the arm is physically at the targetState
     */
    public boolean hasReachedTargetPosition() {
        return shoulder.hasReachedTargetPosition() && elbow.hasReachedTargetPosition() && wrist.hasReachedTargetPosition();
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
        SmartDashboard.putString(targetPositionKey, getTargetState().toString());
        // SmartDashboard.putNumber("Elbow offset", elbowOffset);
        // SmartDashboard.putNumber("Shoulder offset", shoulderOffset);
        // SmartDashboard.putNumber("Wrist offset", wristOffset);
    }

    /**
     * Gets the closest state, using angles.
     * It prefers prefers to move smaller joints than larger ones
     * (i.e. it would rather choose to have the elbow be off by 20
     * degrees than the shoulder)
     * @return
     */
    public ArmState getClosestState() {
        double shoulderAngle = shoulder.getCurrentAngleInDegrees();
        double elbowAngle = elbow.getCurrentAngleInDegrees();
        double wristAngle = wrist.getCurrentAngleInDegrees();

        ArmState closestState = ArmState.STOWED;
        double closestStateDistance = 10000; // Intentionally high number. The first state checked will be less

        for (ArmState state : ArmState.values()) {
            double shoulderDistance = Math.abs(state.getShoulderAngle() - shoulderAngle);
            double elbowDistance = Math.abs(state.getElbowAngle() - elbowAngle);
            double wristDistance = Math.abs(state.getWristAngle() - wristAngle);

            double stateDistance = (shoulderDistance * 1.5) + elbowDistance * (wristDistance * 0.5);
            if(stateDistance < closestStateDistance) {
                closestStateDistance = stateDistance;
                closestState = state;
            }
        }
        return closestState;
    }

    public void clearSetpoint() {
        shoulder.clearSetpoint();
        elbow.clearSetpoint();
        wrist.clearSetpoint();

    }

    public Shoulder getShoulder() {
        return shoulder;
    }

    public Elbow getElbow() {
        return elbow;
    }

    public Wrist getWrist() {
        return wrist;
    }

    /**
     * Helper class for pathfinding algorithm
     */
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
}
