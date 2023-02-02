package frc.team670.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;
import com.revrobotics.REVLibError;

public class Arm extends MustangSubsystemBase {
    private static final double MAX_DEPTH = 10;
    public static final int NUM_STATES = 9;
    private Shoulder shoulder;
    private Elbow elbow;
    private ArmState currentState;

    private static final ArmState[][] VALID_PATHS_GRAPH = new ArmState[][] {
            { ArmState.DOUBLE_SUBSTATION, ArmState.HIGH_SHELF}, // STOWED
            { ArmState.INTERMEDIATE_HOPPER }, // HOPPER
            { ArmState.INTAKE_GROUND, ArmState.HYBRID}, // INTERMEDIATE_HOPPER
            { ArmState.INTERMEDIATE_HOPPER, ArmState.HIGH_SHELF }, // SCORE_CONE_HIGH
            { ArmState.INTERMEDIATE_HOPPER }, // SCORE_CONE_HIGH
            { ArmState.SCORE_CONE_HIGH }, // HIGH_SHELF
            { ArmState.SCORE_CONE_MID, ArmState.INTAKE_GROUND, ArmState.HOPPER }, // HYBRID
            { ArmState.INTERMEDIATE_HOPPER, ArmState.SCORE_CONE_MID }, // INTAKE_GROUND
            { ArmState.INTAKE_GROUND }, // DOUBLE_SUBSTATION
    };

    private static ArmState VALID_PATHS[][][] = new ArmState[NUM_STATES][NUM_STATES][];

    public Arm() {
        //this.shoulder = new Shoulder();
        this.elbow = new Elbow();
        init();
    }

    private static void init() {
        SmartDashboard.putNumber("arm Target ID", 0);
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
        shoulder.debugSubsystem();
        elbow.debugSubsystem();
        moveToTarget(ArmState.getVal((int) SmartDashboard.getNumber("arm Target ID", 0)));
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
        
        SmartDashboard.putNumber("shoulder target (deg)", target.getShoulderAngle());
        SmartDashboard.putNumber("elbow target (deg)", target.getElbowAngle());

    }

    /**
     * Returns the state we're moving towards
     * Ex: If we're moving from A to B, this returns B
     */
    public ArmState getCurrentState() {
        
        return currentState; 
    }

    /**
     * Returns whether or not the arm is physically at the given state
     * 
     * @param target The target state we're checking
     */
    public boolean isAt(ArmState target) {
        return shoulder.hasReachedTargetPosition()&& elbow.hasReachedTargetPosition();
    }

    /**
     * Returns a valid list of states from the starting State to the ending State
     * This list should NOT include the starting state, but SHOULD include the
     * ending state
     */
    public static ArmState[] getValidPath(ArmState start, ArmState finish) {
        // retrieve the list of intermediate states from VALID_PATHS
        
        
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
        //shoulder.debugSubsystem();
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
