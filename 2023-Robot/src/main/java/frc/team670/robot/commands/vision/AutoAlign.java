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
import frc.team670.robot.commands.drivebase.MoveToPosePID;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Vision;

/**
 * AutoAlign
 */
public class AutoAlign extends InstantCommand implements MustangCommand {

    private VisionSubsystemBase vision;
    private SwerveDrive swerve;
    private MustangScheduler scheduler = MustangScheduler.getInstance();

    public AutoAlign(VisionSubsystemBase vision, SwerveDrive swerve) {
        this.vision = vision;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        // find pose of nearest target
        Pose2d robotPose = swerve.getPose();
        var result = vision.getCameras()[0].getLatestResult();
        if (!result.hasTargets()) return;

        var camToTarget = result.getBestTarget().getBestCameraToTarget();
        Transform2d transform = new Transform2d(camToTarget.getTranslation().toTranslation2d(),
                camToTarget.getRotation().toRotation2d());
        Transform2d camOffset2d = new Transform2d(RobotConstants.CAMERA_OFFSET.getTranslation().toTranslation2d(), 
        RobotConstants.CAMERA_OFFSET.getRotation().toRotation2d());
        Pose2d cameraPose = robotPose.transformBy(camOffset2d.inverse());
        Pose2d targetPose = cameraPose.transformBy(transform);

        // transform by offset (to not crash)
        Pose2d goalPose = targetPose.transformBy(RobotConstants.GRID_TO_TARGET_OFFSET);
        SmartDashboard.putNumber("goal pose x", goalPose.getX());
        SmartDashboard.putNumber("goal pose y", goalPose.getY());

        double distance = getDistance(transform.getX(), transform.getY());
        SmartDashboard.putNumber("goal pose distance", distance);

        scheduler.schedule(new MoveToPose(swerve, goalPose, false), swerve);
        // String command = "";
        // if (distance < 1)
        // // command = "PID";
        // scheduler.schedule(new MoveToPosePID(swerve, goalPose, false), swerve);
        // else
        //     scheduler.schedule(new MoveToPose(swerve, goalPose, false), swerve);
        // // command = "path planner";
        // SmartDashboard.putString("move to pose type: ", command);
        // move to target pose, make sure dont crash into target

    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }

    private double getDistance(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }
}
