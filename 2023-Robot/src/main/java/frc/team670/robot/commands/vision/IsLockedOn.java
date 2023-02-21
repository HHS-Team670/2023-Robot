package frc.team670.robot.commands.vision;

import java.util.ArrayList;
import java.util.Map;

import javax.print.attribute.HashAttributeSet;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * IsLockedOn
 */
public class IsLockedOn extends CommandBase implements MustangCommand {

    private SwerveDrive driveBase;
    private VisionSubsystemBase vision;
    private ArrayList<Pose2d> scoringPoses = new ArrayList<>();
    private Pose2d currentPose = null;

    public IsLockedOn(SwerveDrive driveBase, VisionSubsystemBase vision, Pose2d targetPose) {
        this.driveBase = driveBase;
        this.vision = vision;
        addRequirements(vision);

        for (AprilTag tag : FieldConstants.APRILTAGS) 
            scoringPoses.add(tag.pose.toPose2d());
    }

    public IsLockedOn(SwerveDrive driveBase, VisionSubsystemBase vision, int bruh) {
        this.driveBase = driveBase;
        this.vision = vision;
        addRequirements(driveBase, vision);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    @Override
    public void initialize() {
        currentPose = driveBase.getPose();
    }
    
    @Override
    public void execute() {
        SmartDashboard.putBoolean("isLockedOn", false);        
    }
    
    @Override
    public boolean isFinished() {
        
        // if (goalPose != null) {
        //     // SmartDashboard.putString("Target Pose", goalPose.getX() + "," + goalPose.getY());
        //     // SmartDashboard.putString("Current Pose", currentPose.getX() + "," + currentPose.getY());
        //     // SmartDashboard.putNumber("Diff x", Math.abs(currentPose.getX() - goalPose.getX()));
        //     // SmartDashboard.putNumber("Diff y", Math.abs(currentPose.getY() - goalPose.getY()));
        //     // SmartDashboard.putNumber("Diff rot", Math.abs(currentPose.getRotation().getDegrees()
        //     // - goalPose.getRotation().getDegrees()));

        //     if (Math.abs(currentPose.getX() - goalPose.getX()) <= 0.3
        //             && Math.abs(currentPose.getY() - goalPose.getY()) <= 0.3
        //             && Math.abs(currentPose.getRotation().getDegrees()
        //                     - goalPose.getRotation().getDegrees() - 180) <= 10) {   // TODO: change angles based on camera
        //         return true;
        //     }
        // }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("isLockedOn", true);
        // vision.switchLEDS(true, true);
    }

}
