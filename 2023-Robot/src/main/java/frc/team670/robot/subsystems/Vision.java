package frc.team670.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.RobotConstants;

public class Vision extends VisionSubsystemBase {

    public Vision(PowerDistribution pd) {
        super(pd);
        setName("Vision");
        setCamera(RobotConstants.VISION_CAMERA_NAME);
        super.getCamera().setDriverMode(true);
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub
        // super.processImage(0, 0, 0); //TODO: find camera offsets
    }

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
