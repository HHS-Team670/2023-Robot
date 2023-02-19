package frc.team670.robot.subsystems;

import java.util.Arrays;
import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;

public class Vision extends VisionSubsystemBase {

    public Vision(PowerDistribution pd) {
        super(pd, new AprilTagFieldLayout(Arrays.asList(FieldConstants.APRILTAGS), FieldConstants.LENGTH, FieldConstants.WIDTH),
                new PhotonCamera[] {new PhotonCamera(RobotConstants.VISION_CAMERA_NAME)},
                new Transform3d[] {RobotConstants.CAMERA_OFFSET});
        setName("Vision");
        getCameras()[0].setDriverMode(true);
    }

    @Override
    public void mustangPeriodic() {}

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub

    }

    // public Transform2d getTransformationToBestTarget() {
    //     PhotonPipelineResult result = camera.getLatestResult();
    //     if (result.hasTargets()) {
    //         PhotonTrackedTarget target = result.getBestTarget();
    //         Transform3d cameraToTarget = target.getBestCameraToTarget();
    //         cameraToTarget.getTranslation().toTranslation2d();
    //         Rotation3d rotation = cameraToTarget.getRotation();
    //         double x = cameraToTarget.getX();
    //         double y = cameraToTarget.getY();

            
            
    //         return new Transform2d(new Translation2d(x, y), rotation);
    //     } else return new Transform2d();
    // }

}
