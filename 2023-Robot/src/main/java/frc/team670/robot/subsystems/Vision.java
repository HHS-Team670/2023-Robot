package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;

public class Vision extends VisionSubsystemBase {

    public Vision(PowerDistribution pd) {

        super(pd, Vision.getFieldLayout(FieldConstants.aprilTags),
                new PhotonCamera[] {new PhotonCamera(RobotConstants.VISION_CAMERA_NAMES[0]),
                        new PhotonCamera(RobotConstants.VISION_CAMERA_NAMES[1])},
                RobotConstants.CAMERA_OFFSETS);
        setName("Vision");
    }

    @Override
    public void mustangPeriodic() {}

    @Override
    public void debugSubsystem() {}

    private static AprilTagFieldLayout getFieldLayout(Map<Integer, Pose3d> tags) {
        List<AprilTag> t = new ArrayList<>();
        FieldConstants.aprilTags.forEach((i, p) -> {
            t.add(new AprilTag(i, p));
        });
        return new AprilTagFieldLayout(t, FieldConstants.fieldLength, FieldConstants.fieldWidth);
    }
}
