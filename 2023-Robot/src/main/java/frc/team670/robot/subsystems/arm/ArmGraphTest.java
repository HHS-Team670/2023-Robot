package frc.team670.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;

public class ArmGraphTest {
    private static final int MAX_DEPTH = 10;
    private static final int NUM_STATES = 10;
    private static final ArmState[][] VALID_PATHS_GRAPH = new ArmState[][] {
        { ArmState.DOUBLE_SUBSTATION, ArmState.HIGH_SHELF }, // STOWED
        { ArmState.INTERMEDIATE_HOPPER }, // HOPPER
        { ArmState.INTAKE_GROUND, ArmState.HYBRID}, // INTERMEDIATE_HOPPER
        { ArmState.INTERMEDIATE_HOPPER, ArmState.HIGH_SHELF }, // SCORE_CONE_HIGH
        { ArmState.INTERMEDIATE_HOPPER }, // SCORE_CONE_HIGH
        { ArmState.SCORE_CONE_HIGH }, // HIGH_SHELF
        { ArmState.SCORE_CONE_MID, ArmState.INTAKE_GROUND, ArmState.HOPPER }, // HYBRID
        { ArmState.INTERMEDIATE_HOPPER, ArmState.SCORE_CONE_MID }, // INTAKE_GROUND
        { ArmState.INTAKE_GROUND }, // DOUBLE_SUBSTATION
        //{ ArmState.SCORE_CONE_HIGH }
    };
    private static ArmState VALID_PATHS[][][] = new ArmState[NUM_STATES][NUM_STATES][];
    static{
        init();
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



    private static void init() {

        for (int i = 0; i < NUM_STATES; i++) {
            for (int j = 0; j < NUM_STATES; j++) {
                // set validpaths[i][j] to the path between i and j
                VALID_PATHS[i][j] = getValidPath(ArmState.getVal(i), ArmState.getVal(j));
            }
        }
    }

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


    public static void main(String[] args) {
        // ArmGraphTest test = new ArmGraphTest();
        // ArmState[] path1 = ArmGraphTest.getValidPath(ArmState.SCORE_CONE_MID, ArmState.SCORE_CONE_HIGH);
        // ArmState[] path2 = ArmGraphTest.getValidPath(ArmState.ZERO, ArmState.SCORE_CONE_MID);
        // ArmState[] path3 = ArmGraphTest.getValidPath(ArmState.ZERO, ArmState.ZERO);
        // ArmState[] path4 = ArmGraphTest.getValidPath(ArmState.INTAKE_GROUND, ArmState.DOUBLE_SUBSTATION);
        // ArmState[] path5 = ArmGraphTest.getValidPath(ArmState.HIGH_SHELF, ArmState.SCORE_CONE_MID);
        // ArmState[] path6 = ArmGraphTest.getValidPath(ArmState.HIGH_SHELF, ArmState.STOWED);
        // for( int i=0;i<ArmGraphTest.NUM_STATES;i++){
        //     for(int j=0;j<ArmGraphTest.NUM_STATES;j++){
        //         System.out.println(Arrays.toString(ArmGraphTest.getValidPath(ArmState.getVal(i), ArmState.getVal(j))));
        //     }
        // }
        // Logger.consoleLog(Arrays.toString(path1));
        // Logger.consoleLog(Arrays.toString(path2));
        // Logger.consoleLog(Arrays.toString(path3));
        // Logger.consoleLog(Arrays.toString(path4));
        // Logger.consoleLog(Arrays.toString(path5));
        // Logger.consoleLog(Arrays.toString(path6));
        
        
    }
}
