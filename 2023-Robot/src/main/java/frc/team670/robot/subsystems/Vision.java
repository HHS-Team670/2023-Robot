package frc.team670.robot.subsystems;

import org.photonvision.PhotonCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.commands.vision.IsLockedOn;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;

public class Vision extends VisionSubsystemBase {

    public Vision(PowerDistribution pd) {
        super(pd, FieldConstants.VISION_FIELD_LAYOUT, new Pair<>(
                new PhotonCamera(RobotConstants.VISION_CAMERA_NAME), RobotConstants.CAMERA_OFFSET));
        setName("Vision");
        getCameras()[0].setDriverMode(true);
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
}
