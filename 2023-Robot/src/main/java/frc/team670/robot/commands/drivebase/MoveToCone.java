package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.swervelib.pathplanner.MustangPPSwerveControllerCommand;
import frc.team670.mustanglib.utils.MustangController;
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
    private MustangController mController;
    private final static double kSensitivity=0.5;
    private final static double kDeadband=0.1;

    public MoveToCone(DriveBase driveBase,MustangController mController ) {
        this.driveBase = driveBase;
        this.mController=mController;
        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(driveBase, HealthState.GREEN);
        SmartDashboard.putNumber("Target Decimal",0);
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
    @Override
    public void execute(){
        driveBase.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,angleToCone()*kSensitivity,driveBase.getGyroscopeRotation()));
    }

    public boolean isFinished(){
        return mController.getRightStickX()>kDeadband||mController.getRightStickY()>kDeadband||mController.getLeftStickX()>kDeadband||mController.getLeftStickY()>kDeadband;
    }



    public boolean hasTarget(){
        return colorCam.getLatestResult().getTargets().size() > 0;
    }
    //
    public double angleToCone(){
        var result = colorCam.getLatestResult();
        SmartDashboard.putBoolean("Target Present",result.hasTargets());
        if(result.hasTargets()){
            double maxArea=0;
            PhotonTrackedTarget bestTarget=null;
            for (var target:result.getTargets()){
                if(target.getArea()>maxArea){
                    maxArea=target.getArea();
                    bestTarget=target;
                }
            }
            SmartDashboard.putNumber("Target Decimal",bestTarget.getYaw()/60);
            return bestTarget.getPitch()/60;
        }
           
        else
            return 0;
            
    }


}
