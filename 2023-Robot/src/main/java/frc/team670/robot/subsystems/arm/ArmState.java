package frc.team670.robot.subsystems.arm;

public enum ArmState {

    STOWED(0, 11, 28), //Same as hopper FOR NOW
    HOPPER(1,11, 28),
    INTERMEDIATE_HOPPER(2,11,36), //Must check later
    SCORE_CONE_MID(3, 42, 46),
    SCORE_CONE_HIGH(4, 51, 57),
    
    //Intermediate Hooper
    HIGH_SHELF(5, 51, 57),
    HYBRID(6, 25,12), //Check later as well
    
    //Intake ground
    INTAKE_GROUND(7,25, 12),
    
    //doublesubstationintake
    DOUBLE_SUBSTATION(8,42,46); //Needs tuning later
    // //ZERO(9,0,0);
    // STOWED(0, 225, 30), //Same as hopper FOR NOW
    // HOPPER(1,225, 30),
    // INTERMEDIATE_HOPPER(2,225,45), //Must check later
    // SCORE_CONE_MID(3, 170, 100),
    // SCORE_CONE_HIGH(4, 135, 180),
    
    // //Intermediate Hooper
    // HIGH_SHELF(5, 135, 180),
    // HYBRID(6, 110,80), //Check later as well
    
    // //Intake ground
    // INTAKE_GROUND(7,110, 80),
    
    // //doublesubstationintake
    // DOUBLE_SUBSTATION(8,170,100); //Needs tuning later
    // //ZERO(9,0,0);
    

    private int stateID;
    private double shoulderAngle;
    private double elbowAngle;

    // private double wristAngle;

    private ArmState(int stateID, double xPos, double yPos) {
      //private ArmState(int stateID, double shoulderPos, double elbowPos) {
        // this.shoulderAngle=shoulderPos;
        // this.elbowAngle=elbowPos;
        this.stateID = stateID;

      double L1 = 20.0;
      double L2 = 25.0;
      double L3 = 35;
      

      //arm segment lengths
      //20"(to bottomof drive base tubes), 25", 35"
      
    
      double yDisplacement = yPos - L1;

      double q2 =Math.toDegrees(-Math.acos((xPos*xPos + yDisplacement*yDisplacement - L2*L2 - L3*L3) / (2 * L2 * L3)));

      double q1 = Math.toDegrees(Math.atan(yDisplacement/xPos)) + Math.toDegrees(Math.atan((L2 * Math.toDegrees(Math.sin(q2)))/(L2 + L3 * Math.toDegrees(Math.cos(q2)))));
      shoulderAngle = q1 + 90;
      elbowAngle = 180+ q2;
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
        //case 9:
          //return ZERO;
        default:
          return STOWED;
       
      }
    }
  }