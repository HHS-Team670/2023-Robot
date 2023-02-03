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
    public static final int SHOULDER_LEADER_MOTOR = 2;
    public static final int SHOULDER_FOLLOWER_MOTOR = 3;
    public static final int ELBOW_MOTOR = 4;
    public static final int SHOULDER_ABSOLUTE_ENCODER = 0;
    public static final int ELBOW_ABSOLUTE_ENCODER = 1;
  
    // Claw
    public static final int SOLENOID_0 = 0;
    public static final int SOLENOID_1 = 1;
    public static final int CLAW_PUSH_SOLENOID = 2;
  
    public static final int PCM_MODULE = 12;
  
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

}
