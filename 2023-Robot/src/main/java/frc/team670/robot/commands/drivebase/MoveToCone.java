package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.swervelib.pathplanner.MustangPPSwerveControllerCommand;
import frc.team670.robot.commands.vision.IsLockedOn;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.drivebase.DriveBase;

/**
 * MoveToPose - moves to specified pose. Cancels when button is released.
 */
public class MoveToCone extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    // private final Pose2d endPose;
    private PhotonCamera colorCam;
    // private boolean backOut = false;

    protected Map<MustangSubsystemBase, HealthState> healthReqs;


    public MoveToCone(DriveBase driveBase) {
        this.driveBase = driveBase;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
    }

    // public MoveToPose(DriveBase driveBase, Pose2d endPose, boolean backOut) {
    // this.driveBase = driveBase;
    // this.endPose = endPose;
    // this.backOut = backOut;
    // this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
    // this.healthReqs.put(driveBase, HealthState.GREEN);
    // }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        colorCam = driveBase.getPoseEstimator().getVision().getCameras()[1];
    }


    public boolean hasTarget(){
        return colorCam.getLatestResult().getTargets().size() > 0;
    }
    //
    public double angleToCone(){
        var result = colorCam.getLatestResult();
        if(result.hasTargets())
            return colorCam.getLatestResult().getTargets().get(0).getPitch()/60;
        else
            return 0;
            
    }


}
