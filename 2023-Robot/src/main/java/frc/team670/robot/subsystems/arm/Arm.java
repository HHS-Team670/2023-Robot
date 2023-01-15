 package frc.team670.robot.subsystems.arm;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.SparkMaxLite;

public class Arm extends MustangSubsystemBase{
    Shoulder shoulder;
    Elbow elbow;

    ArrayList<ArmState> targetQueue = new ArrayList<ArmState>();

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
        //if we're at the target, then remove from queue

        //if the queue isn't empty, move towards the first item

        //if the queue is empty, do nothing
    }

    public void addToQueue(ArmState target) {
        //use getValidPath to determine the states between the current state and the target state
        //for every state in that list, add to queue using targetQueue.add()   
    }
    /*
     * Returns a valid list of states from the starting State to the ending State
     * This list should NOT include the starting state, but SHOULD include the ending state
     */
    private ArmState[] getValidPath(ArmState start, ArmState finish) {
        //retrieve the list of intermediate states from VALID_PATHS
        //Add the final ArmState to this list
        //return this list
    }

    @Override
    public void debugSubsystem() {
        shoulder.debugSubsystem();
        elbow.debugSubsystem();
    }

}