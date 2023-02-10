package frc.team670.robot.subsystems.arm;

/**
 * In order to add an ArmState
* 1) Add ArmState to enum
 * 2) Add to VALID_PATHS_GRAPH in Arm.java, along with all valid connections
 * 3) Add the state to the valid connections (i.e. if you can go from NEW_STATE to STOWED, then add NEW_STATE to STOWED's neighbor list)
 */

public enum ArmState {

    //Position-based states (x, y) coordinates
    // STOWED(0, 11, 28), //Same as hopper FOR NOW
    // HOPPER(1,11, 28),
    // INTERMEDIATE_HOPPER(2,11,36), //Must check later
    // SCORE_MID(3, 42, 46),
    // SCORE_HIGH(4, 51, 57),
    // HYBRID(5, 25,12), //Check later as well
    // INTAKE_GROUND(6,25, 12),
    // DOUBLE_SUBSTATION(7,42,46); //Needs tuning later

    //Angle-based states. For the shoulder, straight up is 180. If the elbow joint is straight, that's 180.
    //As of 2/4, these are estimates
    STOWED(0, 180, 40),
    HOPPER(1,180, 40),
    INTERMEDIATE_HOPPER(2,225,45),
    SCORE_MID(3, 170, 100),
    SCORE_HIGH(4,135, 180),
    HYBRID(5, 110,80),
    INTAKE_GROUND(6,110, 80),
    DOUBLE_SUBSTATION(7,170,100);
    

    private int stateID;
    private double shoulderAngle;
    private double elbowAngle;
    // private double wristAngle;

    //private ArmState(int stateID, double xPos, double yPos) {
    private ArmState(int stateID, double shoulderPos, double elbowPos) {
      this.shoulderAngle=shoulderPos;
      this.elbowAngle=elbowPos;
      this.stateID = stateID;


      // double L1 = 20.0; // From floor to shoulder joint
      // double L2 = 25.0; // From shoulder joint to elbow joint
      // double L3 = 35; // From elbow joint to mounting point of claw
      

      //arm segment lengths
      //20"(to bottomof drive base tubes), 25", 35"
      
    
      // double yDisplacement = yPos - L1;

      // double q2 =Math.toDegrees(-Math.acos((xPos*xPos + yDisplacement*yDisplacement - L2*L2 - L3*L3) / (2 * L2 * L3)));

      // double q1 = Math.toDegrees(Math.atan(yDisplacement/xPos)) + Math.toDegrees(Math.atan((L2 * Math.toDegrees(Math.sin(q2)))/(L2 + L3 * Math.toDegrees(Math.cos(q2)))));
      // shoulderAngle = q1;
      // elbowAngle = q2;
      //double q2 = -Math.acos((Math.pow(xPos,2) + Math.pow(yDisplacement,2) - Math.pow(L2,2) - Math.pow(L3,2))/(2*L2*L3)); 
      //double q1 = Math.atan(yDisplacement/xPos)+ Math.atan(L3*Math.sin(q2)/(L2 + L3*Math.cos(q2)));
      // shoulderAngle = Math.toDegrees(q1) + 90;
      // elbowAngle = 180 + Math.toDegrees(q2); //q2 should be negative

    }
    
    /*
     * 
     * @return the corresponding integer code for each color on the wheel
     */
    public int getStateID() {
      return stateID;
    }

    public double getShoulderAngle() {
      return shoulderAngle;
    }

    public double getElbowAngle() {
      return elbowAngle;
    }
    
    
    // public double getWristAngle() {
    //   return wristAngle;
    // }

    public static ArmState getVal(int stateID){

      ArmState[] values=ArmState.values();
      for(ArmState state: values){
        if(state.stateID==stateID){
          return state;
        }
      }
      return ArmState.STOWED;
      
    }
  }