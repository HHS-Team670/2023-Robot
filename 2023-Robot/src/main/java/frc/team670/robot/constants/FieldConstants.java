package frc.team670.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class FieldConstants {    // TODO: set field layout
    public static final double LENGTH = 10; 
    public static final double WIDTH = 10;
    public static final AprilTagFieldLayout VISION_FIELD_LAYOUT =
            new AprilTagFieldLayout(null, LENGTH, WIDTH);  
}
