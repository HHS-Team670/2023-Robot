package frc.team670.robot.commands.vision;

import java.util.Map;

import javax.print.attribute.HashAttributeSet;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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

    public IsLockedOn(SwerveDrive driveBase, VisionSubsystemBase vision) {
        this.driveBase = driveBase;
        this.vision = vision;
        addRequirements(driveBase, vision);
    }

    public IsLockedOn(SwerveDrive driveBase, VisionSubsystemBase vision, Pose2d goalPose) {
        this.driveBase = driveBase;
        this.vision = vision;
        addRequirements(driveBase, vision);
        this.goalPose = goalPose;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        SmartDashboard.putBoolean("isLockedOn", false);
        // find pose of nearest target
        currentPose = driveBase.getPose();
        PhotonPipelineResult result = vision.getCameras()[0].getLatestResult(); 
        SmartDashboard.putBoolean("has targets", result.hasTargets());
        if (!result.hasTargets()) return;

        Pose2d targetPose;
        
        if (goalPose == null) {
            Transform3d camToTarget = result.getBestTarget().getBestCameraToTarget();
            Transform2d transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(),
                camToTarget.getRotation().toRotation2d());
            Pose2d cameraPose = currentPose.transformBy(new Transform2d(RobotConstants.CAMERA_OFFSET.inverse().getTranslation().toTranslation2d(), 
                RobotConstants.CAMERA_OFFSET.inverse().getRotation().toRotation2d()));
            targetPose = cameraPose.transformBy(transform);
        } else {
            targetPose = goalPose;
        }
    
        // transform by offset (to not crash)
        goalPose = targetPose.transformBy(RobotConstants.GRID_TO_TARGET_OFFSET);
        // SmartDashboard.putNumber("goal pose x", goalPose.getX());
        // SmartDashboard.putNumber("goal pose y", goalPose.getY());
    }
    
    @Override
    public boolean isFinished() {
        
        if (goalPose != null) {
            // SmartDashboard.putString("Target Pose", goalPose.getX() + "," + goalPose.getY());
            // SmartDashboard.putString("Current Pose", currentPose.getX() + "," + currentPose.getY());
            // SmartDashboard.putNumber("Diff x", Math.abs(currentPose.getX() - goalPose.getX()));
            // SmartDashboard.putNumber("Diff y", Math.abs(currentPose.getY() - goalPose.getY()));
            // SmartDashboard.putNumber("Diff rot", Math.abs(currentPose.getRotation().getDegrees()
            // - goalPose.getRotation().getDegrees()));

            if (Math.abs(currentPose.getX() - goalPose.getX()) <= 0.3
                    && Math.abs(currentPose.getY() - goalPose.getY()) <= 0.3
                    && Math.abs(currentPose.getRotation().getDegrees()
                            - goalPose.getRotation().getDegrees() - 180) <= 10) {   // TODO: change angles based on camera
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("isLockedOn", true);
        // vision.switchLEDS(true, true);
    }

}
