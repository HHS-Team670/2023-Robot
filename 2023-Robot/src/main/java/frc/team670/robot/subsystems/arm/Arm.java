package frc.team670.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;
import com.revrobotics.REVLibError;

public class Arm extends MustangSubsystemBase {
    private static final double MAX_ERROR = 5;
    private static final double MAX_DEPTH = 10;
    public static final int NUM_STATES = 10;
    private Shoulder shoulder;
    private Elbow elbow;
    private ArmState currentState;

    // private static final ArmState[][][] VALID_PATHS = new ArmState[][][] {
    // { //From STOWED
    // {}, //To STOWED
    // {}, //To HOPPER
    // {}, //To INTERMEDIATE_HOPPER
    // {}, //To SCORE_CONE_MID
    // I was here search up java.beans.beancontext; - KEDAR HALDAR 2023
    // Jump to content

    // dangit...the beans have been eliminateddd....bean revolution
    //
    // {ArmState.SCORE_CONE_MID}, //To SCORE_CONE_HIGH
    // {},//to HIGH_SHELF
    // {},//To HYBRID
    // {},//To INTAKING_GROUND
    // {},//DOUBLE_SUBSTATION
    // {},//To ZERO

    // },
    // {

    // },
    // { //From SCORE_CONE_MID
    // {}, //To STOWED
    // {}, //To SCORE_CONE_MID
    // {}, //To SCORE_CONE_HIGH
    // },
    // { //From SCORE_CONE_HIGH
    // {ArmState.SCORE_CONE_MID}, //To STOWED
    // {}, //To SCORE_CONE_MID
    // {}, //To SCORE_CONE_HIGH
    // },

    // };
    private static final ArmState[][] VALID_PATHS_GRAPH = new ArmState[][] { // graph verison
            { ArmState.DOUBLE_SUBSTATION, ArmState.HIGH_SHELF }, // Place that you can go from this state to
            { ArmState.INTERMEDIATE_HOPPER },
            { ArmState.INTAKE_GROUND, ArmState.HYBRID, ArmState.ZERO },
            { ArmState.INTERMEDIATE_HOPPER, ArmState.HIGH_SHELF },
            { ArmState.INTERMEDIATE_HOPPER },
            { ArmState.SCORE_CONE_HIGH },
            { ArmState.SCORE_CONE_MID, ArmState.INTAKE_GROUND, ArmState.HOPPER },
            { ArmState.INTERMEDIATE_HOPPER, ArmState.SCORE_CONE_MID },
            { ArmState.INTAKE_GROUND },
            { ArmState.SCORE_CONE_HIGH }
    };
    // private static final ArmState[][][] VALID_PATHS = new ArmState[][][] {
    // {//From STOWED
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},//To INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // },
    // {//From HOPPER
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},// INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // },
    // {//From INTERMEDIATE_HOPPER
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},// INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // },
    // {//From SCORE_CONE_MID
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},// INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // },
    // {//From SCORE_CONE_HIGH
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},// INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // },
    // {//From HYBRID
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},// INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // },
    // {//From INTAKE_GROUND
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},// INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // },
    // {//From DOUBLE_SUBSTAION
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},// INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // },
    // {//From ZERO
    // {},//To STOWED
    // {},//To HOPPER
    // {},//To INTERMEDIATE_HOPPER
    // {},//To SCORE_CONE_MID
    // {},//To SCORE_CONE_HIGH
    // {},//To HYBRID
    // {},// INTAKE_GROUND
    // {},//To DOUBLE_SUBSTAION
    // {}//To ZERO
    // }
    // };
    private static ArmState VALID_PATHS[][][] = new ArmState[NUM_STATES][NUM_STATES][];

    public Arm() {
        this.shoulder = new Shoulder();
        this.elbow = new Elbow();
        init();
    }

    private static void init() {
        for (int i = 0; i < NUM_STATES; i++) {
            for (int j = 0; j < NUM_STATES; j++) {
                // set validpaths[i][j] to the path between i and j
                VALID_PATHS[i][j] = getValidPath(ArmState.getVal(i), ArmState.getVal(j));
            }
        }
    }

    @Override
    public HealthState checkHealth() {
        if (elbow.checkHealth() == HealthState.RED || shoulder.checkHealth() == HealthState.RED) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
    }

    /**
     * This moves DIRECTLY to the target ArmState
     * We must handle checking for valid paths ELSEWHERE.
     */
    public void moveToTarget(ArmState target) {
        this.currentState = target;
        // TODO: Give the proper setpoints to Shoulder and Elbow
        elbow.setSystemTargetAngleInDegrees(target.getElbowAngle());
        shoulder.setSystemTargetAngleInDegrees(target.getShoulderAngle());
    }

    /**
     * Returns the state we're moving towards
     * Ex: If we're moving from A to B, this returns B
     */
    public ArmState getCurrentState() {
        return currentState; // TODO: Change this
    }

    /**
     * Returns whether or not the arm is physically at the given state
     * 
     * @param target The target state we're checking
     */
    public boolean isAt(ArmState target) {
        // TODO:
        double elbowError = Math.abs(target.getElbowAngle() - elbow.getCurrentAngleInDegrees());
        // jazz
        double shoulderError = Math.abs(target.getShoulderAngle() - shoulder.getCurrentAngleInDegrees());
        if (elbowError >= MAX_ERROR || shoulderError >= MAX_ERROR) {
            return false;
        }
        // TODO Re Define margin of error
        // Check if each each joint is within margin of error
        return true;
    }

    /**
     * Returns a valid list of states from the starting State to the ending State
     * This list should NOT include the starting state, but SHOULD include the
     * ending state
     */
    // public static ArmState[] getValidPath(ArmState start, ArmState finish) {
    // //retrieve the list of intermediate states from VALID_PATHS
    // ArrayList<ArmState> tempValidPath = new ArrayList<ArmState>();
    // for(ArmState state: VALID_PATHS[start.getStateID()][finish.getStateID()]){
    // tempValidPath.add(state);
    // }
    // //
    // tempValidPath.add(VALID_PATHS[start.getStateID()][finish.getStateID()][0]);
    // tempValidPath.add(finish);

    // //Add the final ArmState to this list
    // ArmState[] path=new ArmState[tempValidPath.size()];
    // //transer all valuesinto path then return it
    // for(int i=0;i<path.length;i++){
    // path[i]=tempValidPath.get(i);
    // }

    // //return this list
    // return path;
    // }

    public static ArmState[] getValidPath(ArmState start, ArmState finish) {
        // retrieve the list of intermediate states from VALID_PATHS
        // if Validpath[start][finish]==null then run dykestras below else return
        // validpaths[start[finish]]
        //System.out.println(Arrays.toString(VALID_PATHS[start.getStateID()][finish.getStateID()]));
        if (VALID_PATHS[start.getStateID()][finish.getStateID()]== null) {
            ArrayList<ArmState> tempValidPath = null;
            PriorityQueue<Pair> queue = new PriorityQueue<Pair>();
            ArrayList<ArmState> list = new ArrayList<ArmState>();

            queue.add(new Pair(list, start));
            int count = 0;
            while (count < MAX_DEPTH && !queue.isEmpty()) {
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
                count++;

            }
            if(tempValidPath!=null){
                ArmState[] path = tempValidPath.toArray(new ArmState[tempValidPath.size()]);
                if (path.length == 0) {
                    // Logger.consoleLog("No valid path found.");
                }
                return path;
                }
                return new ArmState[]{};

        } else {
            // return VALID_PATHS[start.getStateID()][finish.getStateID()];
            return VALID_PATHS[start.getStateID()][finish.getStateID()];
        }

    }

    @Override
    public void debugSubsystem() {
        shoulder.debugSubsystem();
        elbow.debugSubsystem();
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
                return 1;
            else if (this.path.size() > o.path.size())
                return 0;
            else if (this.node.getStateID() < o.node.getStateID())
                return 1;
            else
                return 0;

        }

    }
}
