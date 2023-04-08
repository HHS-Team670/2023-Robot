package frc.team670.robot.subsystems;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;

public class Vision extends VisionSubsystemBase {

    public Vision(PowerDistribution pd) {

        super(pd, FieldConstants.getFieldLayout(FieldConstants.aprilTags),
                new PhotonCamera[] {new PhotonCamera(RobotConstants.Vision.kVisionCameraIDs[0]),
                        new PhotonCamera(RobotConstants.Vision.kVisionCameraIDs[1])},
                RobotConstants.Vision.kCameraOffsets);
        setName("Vision");
    }

    @Override
    public void mustangPeriodic() {}

    @Override
    public void debugSubsystem() {}

}
