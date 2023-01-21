package frc.team670.robot.subsystems.arm;

public enum ArmState {

    STOWED(0, 0.0, 90.0), 
    HOPPER(1,90.0,90.0),
    INTERMEDIATE_HOPPER(2,90.0,90.0),
    SCORE_CONE_MID(3, 90, 90),
    SCORE_CONE_HIGH(4, 90.0, 180.0),
    
    //Intermediate Hooper
    HIGH_SHELF(5,90.0,90.0),
    HYBRID(6,90.0,90.0),

    
    //Intake ground
    INTAKE_GROUND(7,90.0,90.0),
    
    //doublesubstationintake
    DOUBLE_SUBSTATION(8,90,90),
    ZERO(9,0,0);
    
    //zero????
    

    private int stateID;
    private double shoulderAngle;
    private double elbowAngle;
    // private double wristAngle;

    private ArmState(int stateID, double shoulderAngle, double elbowAngle) {
      this.stateID = stateID;
      this.shoulderAngle = shoulderAngle;
      this.elbowAngle = elbowAngle;
      // this.wristAngle= wristAngle;
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
    
    // public double getWristAngle() {
    //   return wristAngle;
    // }
    public static ArmState getVal(int stateID){
      switch(stateID){
        case 0:
          return STOWED;
        case 1:
          return HOPPER;
        case 2:
          return INTERMEDIATE_HOPPER;
        case 3:
          return SCORE_CONE_MID;
        case 4:
          return SCORE_CONE_HIGH;
        case 5:
          return HIGH_SHELF;
        case 6:
          return HYBRID;
        case 7:
          return INTAKE_GROUND;
        case 8:
          return DOUBLE_SUBSTATION;
        case 9:
          return ZERO;
        default:
          return STOWED;
       
      }
    }
  }