package frc.team670.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

public class PoseEstimatorSubsystem extends MustangSubsystemBase {

    private final DriveBase driveBase;
    private final Vision vision;

    /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  public PoseEstimatorSubsystem(Vision vision, DriveBase driveBase) {
    this.vision = vision;
    this.driveBase = driveBase;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =  new SwerveDrivePoseEstimator(driveBase.getSwerveKinematics(),
        driveBase.getGyroscopeRotation(),
        driveBase.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    tab.addString("Pose", this::getFormattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  public void addTrajectory(Trajectory traj)
  {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  @Override
  public void mustangPeriodic() {
    update();
    field2d.setRobotPose(getCurrentPose());
  }

  private void update() {
    EstimatedRobotPose[] visionPoses =
    vision.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    
    for (EstimatedRobotPose pose : visionPoses) {
      if (pose != null) 
      poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(),
      pose.timestampSeconds);
    }
    poseEstimator.update(driveBase.getGyroscopeRotation(), driveBase.getModulePositions());

    Pose2d estPose = poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("est pose x: ", estPose.getX());
    SmartDashboard.putNumber("est pose y: ", estPose.getY());
    SmartDashboard.putNumber("est pose deg: ", estPose.getRotation().getDegrees());
}

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      driveBase.getGyroscopeRotation(),
      driveBase.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

    @Override
    public HealthState checkHealth() {
        return null;
    }

    @Override
    public void debugSubsystem() {
    }

}
