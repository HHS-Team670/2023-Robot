package frc.team670.robot.subsystems;

import org.photonvision.PhotonCamera;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;

/**
 * @author ethan c
 */
public class Vision extends VisionSubsystemBase {
    private static Vision mInstance;

    public static synchronized Vision getInstance() {
        return mInstance == null ? new Vision() : mInstance;
    }

    public Vision() {
        super(FieldConstants.getFieldLayout(FieldConstants.aprilTags),
                new PhotonCamera[] {new PhotonCamera(RobotConstants.Vision.kVisionCameraIDs[0]),
                        new PhotonCamera(RobotConstants.Vision.kVisionCameraIDs[1])},
                RobotConstants.Vision.kCameraOffsets);
        setName("Vision");
    }

    @Override
    public void mustangPeriodic() {
        super.mustangPeriodic();
    }

    @Override
    public void debugSubsystem() {}

}
