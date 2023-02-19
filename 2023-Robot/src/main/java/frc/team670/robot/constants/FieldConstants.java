package frc.team670.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

public class FieldConstants {
    public static final double LENGTH = 16.54175;
    public static final double WIDTH = 8.0137;
    public static final AprilTag[] APRILTAGS = {
        new AprilTag(1, 
            new Pose3d(15.513558, 1.071626, 0.462788, 
            new Rotation3d(
                new Quaternion(0, 0, 1, 0)
                )
            )
        ),
        new AprilTag(2, 
            new Pose3d(15.513558, 2.748026, 0.462788, 
            new Rotation3d(
                new Quaternion(0, 0, 1, 0)
                )
            )
        ),
        new AprilTag(3, 
            new Pose3d(15.513558, 4.424426, 0.462788, 
            new Rotation3d(
                new Quaternion(0, 0, 1, 0)
                )
            )
        ),
        new AprilTag(4, 
            new Pose3d(16.178784, 6.749796, 0.695452, 
            new Rotation3d(
                new Quaternion(0, 0, 1, 0)
                )
            )
        ),
        new AprilTag(5, 
            new Pose3d(0.36195, 6.749796, 0.695452, 
            new Rotation3d(
                new Quaternion(1, 0, 0, 0)
                )
            )
        ),
        new AprilTag(6, 
            new Pose3d(1.02743, 4.424426, 0.462788, 
            new Rotation3d(
                new Quaternion(1, 0, 0, 0)
                )
            )
        ),
        new AprilTag(7, 
            new Pose3d(1.02743, 2.748026, 0.462788, 
            new Rotation3d(
                new Quaternion(1, 0, 0, 0)
                )
            )
        ),
        new AprilTag(8, 
            new Pose3d(1.02743, 1.071626, 0.462788, 
            new Rotation3d(
                new Quaternion(1, 0, 0, 0)
                )
            )
        )
    };
}
