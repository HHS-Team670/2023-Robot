package frc.team670.robot.subsystems.arm;

/**
 * 
 * In order to add an ArmState
*  1) Add ArmState to enum
 * 2) Add the state to the valid connections (i.e. if you can go from NEW_STATE to STOWED, then add NEW_STATE to STOWED's neighbor list)
 */

public enum ArmState {
  
    //Angle-based states. For the shoulder, straight up is 180. If the elbow joint is straight, that's 180.
    
    STOWED(0, 188, 10, 316),

    HYBRID(1, 140, 50, 254),

    SCORE_MID(2, 200, 76, 184), 
    // SCORE_MID(2, 191, 91, 194),  possible score high
    SCORE_HIGH(3, 167, 125, 180),
    STARTING(4, 205,25,270),
    TUNING(5, 90, 180, 180),
    SINGLE_STATION(6, 231, 15, 249),
    INTAKE_SHELF(7, 252, 25, 241),
    UPRIGHT_GROUND(8, 162, 35, 260);
    




  
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
