package frc.team670.robot.commands.vision;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.PoseEstimatorSubsystem;
import frc.team670.robot.subsystems.Vision;

/**
 * AutoAlign - autonomously moves the robot to a given target. If no target is given, it moves to
 * the closest one.`
 */
public class AutoAlign extends InstantCommand implements MustangCommand {

    private VisionSubsystemBase vision;
    private DriveBase driveBase;
    private PoseEstimatorSubsystem poseEstimatorSubsystem;
    private MustangScheduler scheduler = MustangScheduler.getInstance();
    private Pose2d targetPose;
    

    public AutoAlign(VisionSubsystemBase vision, DriveBase driveBase, PoseEstimatorSubsystem poseEstimatorSubsystem) {
        this.vision = vision;
        this.driveBase = driveBase;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        targetPose = null;
    }

    public AutoAlign(VisionSubsystemBase vision, DriveBase driveBase, PoseEstimatorSubsystem poseEstimatorSubsystem, Pose2d targetPose) {
        this.vision = vision;
        this.driveBase = driveBase;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        Pose2d robotPose = poseEstimatorSubsystem.getCurrentPose();

        // find pose of nearest target if none supplied
        if (targetPose == null)
            targetPose = getClosestTargetPose(robotPose);

        // transform by offset (to not crash)
        Pose2d goalPose = targetPose.transformBy(FieldConstants.GRID_TO_TARGET_OFFSET(targetPose));

        scheduler.schedule(new IsLockedOn(driveBase, vision, targetPose), driveBase);
        scheduler.schedule(new MoveToPose(driveBase, poseEstimatorSubsystem, goalPose), driveBase);
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    private Pose2d getClosestTargetPose(Pose2d robotPose) {
        Pose2d targetPose = FieldConstants.CONE_POSES[0];
        for (int i = 1; i < FieldConstants.CONE_POSES.length; i++) {
            double distance = FieldConstants.CONE_POSES[i].getTranslation()
                    .getDistance(robotPose.getTranslation());
            double minDistance =
                    targetPose.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < minDistance)
                targetPose = FieldConstants.CONE_POSES[i];
        }
        for (int i = 0; i < FieldConstants.APRILTAGS.length; i++) {
            double distance = FieldConstants.APRILTAGS[i].pose.toPose2d().getTranslation()
                    .getDistance(robotPose.getTranslation());
            double minDistance =
                    targetPose.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < minDistance)
                targetPose = FieldConstants.APRILTAGS[i].pose.toPose2d();
        }
        return targetPose;
    }
}
