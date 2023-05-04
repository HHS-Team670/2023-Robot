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
        if(mInstance==null){
            mInstance=new Vision();
        }
        return mInstance;
    }

    public Vision() {
        super(RobotConstants.Vision.kConfig);
        setName("Vision");
    }

    @Override
    public void mustangPeriodic() {
        super.mustangPeriodic();
    }

    @Override
    public void debugSubsystem() {}

}
