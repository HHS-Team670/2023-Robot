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
    STOWED(0, 180, 40, 180),
    HYBRID(1, 108, 165, 180),
    SCORE_MID(2, 170, 115, 180),
    SCORE_HIGH(3,120, 203, 180),
    INTERMEDIATE_BACKWARD_GROUND(4, 217, 40, 180),
    BACKWARD_GROUND(5, 217, 310, 180),
    TUNING(6, 90, 180, 180),
    INTERMEDIATE_SCORE(7, 210, 40, 180);
  
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