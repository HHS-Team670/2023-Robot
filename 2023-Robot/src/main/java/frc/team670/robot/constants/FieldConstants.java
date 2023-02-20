package frc.team670.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

public class FieldConstants {
    private static final double coneOffset = 0.4191;
    private static final Rotation3d rRot = new Rotation3d(new Quaternion(0, 0, 1, 0));
    private static final Rotation3d bRot = new Rotation3d(new Quaternion(1, 0, 0, 0));

    public static final double LENGTH = 16.54175;
    public static final double WIDTH = 8.0137;
    public static final AprilTag[] APRILTAGS = {
        new AprilTag(1, new Pose3d(15.513558, 1.071626, 0.462788, rRot)),
        new AprilTag(2, new Pose3d(15.513558, 2.748026, 0.462788, rRot)),
        new AprilTag(3, new Pose3d(15.513558, 4.424426, 0.462788, rRot)),
        new AprilTag(4, new Pose3d(16.178784, 6.749796, 0.695452, rRot)),
        
        new AprilTag(5, new Pose3d(0.36195, 6.749796, 0.695452, bRot)),
        new AprilTag(6, new Pose3d(1.02743, 4.424426, 0.462788, bRot)),
        new AprilTag(7, new Pose3d(1.02743, 2.748026, 0.462788, bRot)),
        new AprilTag(8, new Pose3d(1.02743, 1.071626, 0.462788, bRot))
    };

    public static final Pose2d[] CONE_POSES = {
        new Pose2d(15.513558, 1.071626 - coneOffset, rRot.toRotation2d()),
        new Pose2d(15.513558, 1.071626 + coneOffset, rRot.toRotation2d()),
        new Pose2d(15.513558, 2.748026 - coneOffset, rRot.toRotation2d()),
        new Pose2d(15.513558, 2.748026 + coneOffset, rRot.toRotation2d()),
        new Pose2d(15.513558, 4.424426 - coneOffset, rRot.toRotation2d()),
        new Pose2d(15.513558, 4.424426 + coneOffset, rRot.toRotation2d()),

        new Pose2d(1.02743, 1.071626 - coneOffset, bRot.toRotation2d()),
        new Pose2d(1.02743, 1.071626 + coneOffset, bRot.toRotation2d()),
        new Pose2d(1.02743, 2.748026 - coneOffset, bRot.toRotation2d()),
        new Pose2d(1.02743, 2.748026 + coneOffset, bRot.toRotation2d()),
        new Pose2d(1.02743, 4.424426 - coneOffset, bRot.toRotation2d()),
        new Pose2d(1.02743, 4.424426 + coneOffset, bRot.toRotation2d()),

    };
}
