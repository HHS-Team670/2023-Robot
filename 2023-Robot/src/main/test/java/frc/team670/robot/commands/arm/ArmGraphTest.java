package frc.team670.robot.commands.arm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;

public class ArmGraphTest {
    public static void main(String[] args) {    
        for(int i = 0; i < ArmGraphTest.VALID_PATHS_GRAPH.length; i++){
            for(int j = 0; j < ArmGraphTest.VALID_PATHS_GRAPH.length; j++){
                ArmState[] validPath = ArmGraphTest.getValidPath(ArmState.getVal(i), ArmState.getVal(j));
                System.out.println("From " + ArmState.getVal(i) + " to " + ArmState.getVal(j) + " is " + Arrays.toString(validPath));
            }
        }    
    }
}
