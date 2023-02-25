package frc.team670.robot.subsystems.arm;

/**
 * 
 * In order to add an ArmState
*  1) Add ArmState to enum
 * 2) Add the state to the valid connections (i.e. if you can go from NEW_STATE to STOWED, then add NEW_STATE to STOWED's neighbor list)
 */

public enum ArmState {
  
    //Angle-based states. For the shoulder, straight up is 180. If the elbow joint is straight, that's 180.
    
    //Temporarily set all wrist angles to 180
    // STOWED(0, 205, 38, 58), //normal stowed state
    STOWED(0, 205, 38, 245),
    // STOWED(0, 205, 11, 278), //potential dual-state for stowed and single station
    HYBRID(1, 148, 38, 239),
    SCORE_MID(2, 188, 83, 152),
    SCORE_HIGH(3,135, 180, 117),
    INTERMEDIATE_SCORE(4, 205, 70, 180),
    STARTING(5, 205,25,235),
    TUNING(6, 90, 180, 180),
    INTERMEDIATE_HYBRID(7, 205, 38, 245),
    SINGLE_STATION(8, 244, 10, 221);


  
    private int stateID;
    private double shoulderAngle;
    private double elbowAngle;
    private double wristAngle;

    //private double lowerElbowSoftLimit;
    //private double upperElbowSoftLimit;

    private ArmState(int stateID, double shoulderAngle, double elbowAngle, double wristAngle) {
      this.shoulderAngle = shoulderAngle;
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
      this.stateID = stateID;
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

    public double getWristAngle() {
      return wristAngle;
    }

    // public double getLowerElbowSoftLimit() {
    //   return lowerElbowSoftLimit;
    // }

    // public double getUpperElbowSoftLimit() {
    //   return upperElbowSoftLimit;
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
