package frc.team670.robot.constants;

public class RobotMap {

    public static final int PDP_ID = 0;

    // Drive Base
    public static final int SPARK_LEFT_MOTOR_1 = 20; // These are properly set. 
    public static final int SPARK_LEFT_MOTOR_2 = 21; 
    public static final int SPARK_RIGHT_MOTOR_1 = 22;
    public static final int SPARK_RIGHT_MOTOR_2 = 23;
  
    //Encoders
    //Drivebase
    public static final int LEFT_ENCODER_CHANNEL_A = 0; // These are properly set
    public static final int LEFT_ENCODER_CHANNEL_B = 1;
    public static final int RIGHT_ENCODER_CHANNEL_A = 2;
    public static final int RIGHT_ENCODER_CHANNEL_B = 3;
  
    // Arm
    public static final int ARM_EXTENSION_MOTOR = 8;
    //public static final int ARM_ELBOW_ROTATION_MOTOR_TALON = 6;
    public static final int ARM_ONE_MOTOR = 6;
    public static final int ARM_TWO_MOTOR = 6;
    public static final int ARM_ELBOW_ROTATION_MOTOR_VICTOR = 5;
    public static final int ARM_WRIST_ROTATION = 9;
  
    // Claw
    public static final int SOLENOID_0 = 0;
    public static final int SOLENOID_1 = 1;
    public static final int CLAW_PUSH_SOLENOID = 2;
  
    public static final int PCM_MODULE = 12;
  
  // Climber
    public static final int BACK_CLIMBER_PISTON_CONTROLLER = 14;
    public static final int FRONT_CLIMBER_PISTON_CONTROLLER = 15;
    
    // Sensors
    //public final static Port NAVX_PORT = SerialPort.Port.kUSB;  
  
    public static final int FRONT_ULTRASONIC_TRIGGER_PIN = 8; // TODO set these
    public static final int FRONT_ULTRASONIC_ECHO_PIN = 9;
    public static final int BACK_LEFT_ULTRASONIC_TRIGGER_PIN = 4;
    public static final int BACK_LEFT_ULTRASONIC_ECHO_PIN = 5;
    public static final int BACK_RIGHT_ULTRASONIC_TRIGGER_PIN = 6;
    public static final int BACK_RIGHT_ULTRASONIC_ECHO_PIN = 7;
    public final static int INTAKE_IR_DIO_PORT = 10;
    public final static int INTAKE_BEAM_BREAK_DIO_PORT = 11;
  
  
    // Joysticks
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  
    //Intake
    public static final int INTAKE_BASE_TALON = 7;
    public static final int INTAKE_ROLLER_TALON = 4;
  
    /** PCM Port of the back LED ring (super bright one) */
    public static int BACK_LED_RING = 4;
}
