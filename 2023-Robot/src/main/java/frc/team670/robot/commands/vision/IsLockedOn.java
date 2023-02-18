package frc.team670.robot.commands.vision;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.robot.constants.RobotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * IsLockedOn
 */
public class IsLockedOn extends CommandBase implements MustangCommand {

    private SwerveDrive driveBase;
    private VisionSubsystemBase vision;
    private Pose2d currentPose;
    private Pose2d goalPose;
    private boolean isLockedOn = false;

    public IsLockedOn(SwerveDrive driveBase, VisionSubsystemBase vision) {
        this.driveBase = driveBase;
        this.vision = vision;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void initialize() {
        // find pose of nearest target
        currentPose = driveBase.getPose();
        var result = vision.getCamera().getLatestResult();
        if (!result.hasTargets())
            return;

        var camToTarget = result.getBestTarget().getBestCameraToTarget();
        Transform2d transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(),
                camToTarget.getRotation().toRotation2d());
        Pose2d cameraPose = currentPose.transformBy(RobotConstants.CAMERA_OFFSET.inverse());
        Pose2d targetPose = cameraPose.transformBy(transform);

        // transform by offset (to not crash)
        goalPose = targetPose.transformBy(RobotConstants.GRID_TO_TARGET_OFFSET);
        // SmartDashboard.putNumber("goal pose x", goalPose.getX());
        // SmartDashboard.putNumber("goal pose y", goalPose.getY());

    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("isLockedOn", isLockedOn);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(currentPose.getX() - goalPose.getX()) <= 0.3
                && Math.abs(currentPose.getY() - goalPose.getY()) <= 0.3
                && Math.abs(currentPose.getRotation().getDegrees()
                        - goalPose.getRotation().getDegrees()) <= 4) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        isLockedOn = true;
        SmartDashboard.putBoolean("isLockedOn", isLockedOn);
        vision.switchLEDS(true, true);
    }

}
