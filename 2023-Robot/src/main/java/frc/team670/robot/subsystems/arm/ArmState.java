package frc.team670.robot.subsystems.arm;

public enum ArmState {

    STOWED(0, 0.0, 90.0), 
    SCORE_CONE_MID(1, 90, 90),
    SCORE_CONE_HIGH(2, 90.0, 180.0);

    private int stateID;
    private double shoulderAngle;
    private double elbowAngle;

    private ArmState(int stateID, double shoulderAngle, double elbowAngle) {
      this.stateID = stateID;
      this.shoulderAngle = shoulderAngle;
      this.elbowAngle = elbowAngle;
    }
    
    /**
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
  }