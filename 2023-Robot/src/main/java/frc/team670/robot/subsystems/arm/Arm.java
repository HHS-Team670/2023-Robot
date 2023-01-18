 package frc.team670.robot.subsystems.arm;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

public class Arm extends MustangSubsystemBase{
    private Shoulder shoulder;
    private Elbow elbow;

    private static final ArmState[][][] VALID_PATHS = new ArmState[][][] {
        { //From STOWED
            {}, //To STOWED
            {}, //To SCORE_CONE_MID
            {ArmState.SCORE_CONE_MID}, //To SCORE_CONE_HIGH 
        },
        { //From SCORE_CONE_MID
            {}, //To STOWED
            {}, //To SCORE_CONE_MID
            {}, //To SCORE_CONE_HIGH
        },
        { //From SCORE_CONE_HIGH
            {ArmState.SCORE_CONE_MID}, //To STOWED
            {}, //To SCORE_CONE_MID
            {}, //To SCORE_CONE_HIGH
        },
    };

    public Arm() {
        this.shoulder = new Shoulder();
        this.elbow = new Elbow();
    }

    @Override
    public HealthState checkHealth() {
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
        //TODO: Give the proper setpoints to Shoulder and Elbow
    }

    /**
     * Returns the state we're moving towards
     * Ex: If we're moving from A to B, this returns B
     */
    public ArmState getCurrentState() {
        return null; //TODO: Change this
    }

    /**
     * Returns whether or not the arm is physically at the given state
     * @param target The target state we're checking
     */
    public boolean isAt(ArmState target){
        //TODO:
        //Define margin of error
        // Check if each each joint is within margin of error
        return false;
    }

    /** 
     * Returns a valid list of states from the starting State to the ending State
     * This list should NOT include the starting state, but SHOULD include the ending state
     */
    public static ArmState[] getValidPath(ArmState start, ArmState finish) {
        //retrieve the list of intermediate states from VALID_PATHS
        ArrayList<ArmState> tempValidPath = new ArrayList<ArmState>();
        tempValidPath.add(VALID_PATHS[start.getStateID()][finish.getStateID()][0]);
        tempValidPath.add(finish);

        //Add the final ArmState to this list


        //return this list
        return null;
    }
    

    @Override
    public void debugSubsystem() {
        shoulder.debugSubsystem();
        elbow.debugSubsystem();
    }
}