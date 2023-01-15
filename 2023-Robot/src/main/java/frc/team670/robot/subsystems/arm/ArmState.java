package frc.team670.robot.subsystems.arm;

public enum ArmState {

    STOWED(0, 0.0, 90.0), // 2022 blue game piece
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

    //TODO: Modify to get the 3 fields
    /**
     * 
     * @return the corresponding integer code for each color on the wheel
     */
    public int getColorNumber() {
      return colorNumber;
    }

    private Color getTargetColor() {
      return color;
    }
  }