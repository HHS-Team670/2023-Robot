package frc.team670.robot.subsystems.arm;

/**
 * 
 * In order to add an ArmState
 * 1) Add ArmState to enum
 * 2) Add to VALID_PATHS_GRAPH in Arm.java, along with all valid connections
 * 3) Add the state to the valid connections (i.e. if you can go from NEW_STATE
 * to STOWED, then add NEW_STATE to STOWED's neighbor list)
 */

public enum ArmState {

  // Angle-based states. For the shoulder, straight up is 180. If the elbow joint
  // is straight, that's 180.

  // shoulder second, elbow third\][]
  STOWED(0, 180, 40),
  HYBRID(1, 108, 165),
  SCORE_MID(2, 170, 115), // 163, 113 (FAIL)
  SCORE_HIGH(3, 120, 203), // 120, 197
  INTERMEDIATE_BACKWARD_GROUND(4, 217, 40),
  BACKWARD_GROUND(5, 217, 310),
  TUNING(6, 90, 180),
  INTERMEDIATE_SCORE(7, 210, 40);

  private int stateID;
  private double shoulderAngle;
  private double elbowAngle;
  // private double lowerElbowSoftLimit;
  // private double upperElbowSoftLimit;
  // private double wristAngle;

  // private ArmState(int stateID, double xPos, double yPos) {
  private ArmState(int stateID, double shoulderPos, double elbowPos) {
    this.shoulderAngle = shoulderPos;
    this.elbowAngle = elbowPos;
    this.stateID = stateID;
    // this.lowerElbowSoftLimit = lowerSoftLimit;
    // this.upperElbowSoftLimit = upperSoftLimit;
  }

  // private ArmState(int stateID, double xPos, double yPos) {

  public double getElbowAngle() {
    return elbowAngle;
  }

  // public double getLowerElbowSoftLimit() {
  // return lowerElbowSoftLimit;
  // }

  // public double getUpperElbowSoftLimit() {
  // return upperElbowSoftLimit;
  // }

  // public double getWristAngle() {
  // return wristAngle;
  // }

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

  // return wristAngle;
  // }

  public static ArmState getVal(int stateID) {

    ArmState[] values = ArmState.values();
    for (ArmState state : values) {
      if (state.stateID == stateID) {
        return state;
      }
    }
    return ArmState.STOWED;

  }
}