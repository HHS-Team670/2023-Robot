package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.robot.commands.vision.IsLockedOn;
import frc.team670.robot.constants.RobotConstants;

public class Vision extends VisionSubsystemBase {

    public Vision(PowerDistribution pd) {
        super(pd);
        setName("Vision");
        // setCamera(RobotConstants.VISION_CAMERA_NAME);
        setCamera("Arducam_B");
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

    /*
     * PhotonCameraWrapper from Photonvision docs
     */
    public class PhotonCameraWrapper {
        public PhotonCamera photonCamera;
        public RobotPoseEstimator photonPoseEstimator;

        public PhotonCameraWrapper() {
            // Set up a test arena of two apriltags at the center of each driver station set
            final AprilTag tag18 = new AprilTag(18, new Pose3d(
                new Pose2d(
                    FieldConstants.length,
                    FieldConstants.width / 2.0,
                    Rotation2d.fromDegrees(180)
                )));
            final AprilTag tag01 = new AprilTag(01, new Pose3d(
                new Pose2d(
                    0.0, 
                    FieldConstants.width / 2.0, 
                    Rotation2d.fromDegrees(0.0)
                )));
            ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
            atList.add(tag18);
            atList.add(tag01);

            // TODO - once 2023 happens, replace this with just loading the 2023 field
            // arrangement
            AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

            // Forward Camera
            photonCamera = new PhotonCamera(VisionConstants.cameraName); // Change the name of your camera here to whatever it is in the
            // PhotonVision UI.

            // Create pose estimator
            photonPoseEstimator = new RobotPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.robotToCam);
        }

        /**
         * @param estimatedRobotPose The current best guess at robot pose
         * @return A pair of the fused camera observations to a single Pose2d on the
         *         field, and the time
         *         of the observation. Assumes a planar field and the robot is always
         *         firmly on the ground
         */
        public Optional<Pair<Pose3d, Double>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimator.update();
        }
    }
}
