package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * MoveToPose - moves to specified pose. Cancels when button is released.
 */
public class MoveToCone extends Command implements MustangCommand {
    private DriveBase driveBase;
    // private final Pose2d endPose;
    private PhotonCamera colorCam;
    // private boolean backOut = false;

    protected Map<MustangSubsystemBase, HealthState> healthReqs;
    private MustangController mController;
    private final static double kSensitivity=0.5;


    public MoveToCone(DriveBase driveBase,MustangController mController ){
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
        colorCam = driveBase.getPoseEstimator().getVision().getCameras()[0];
        
        
    }
    @Override
    public void execute(){
        SmartDashboard.putNumber("Theta Velocit",kSensitivity*angleToCone());
        driveBase.setmDesiredHeading(new Rotation2d(driveBase.getPose().getRotation().getRadians() + angleToCone()));    
    }

    public boolean isFinished(){
        
        return !mController.getRightBumper();
    }



    public boolean hasTarget(){
        return colorCam.getLatestResult().getTargets().size() > 0;
    }
    //
    public double angleToCone(){
        var result = colorCam.getLatestResult();
        SmartDashboard.putBoolean("Target Present",result.hasTargets());
        if(result.hasTargets()){
            // double maxArea=0;
            // PhotonTrackedTarget bestTarget=null;
            // for (var target:result.getTargets()){
            //     if(target.getArea()>maxArea){
            //         maxArea=target.getArea();
            //         bestTarget=target;
            //     }
            // }
            // SmartDashboard.putNumber("Target Decimal",result.getTargets().get(0).getYaw()/60);
            return -result.getTargets().get(0).getYaw() * Math.PI / 180;
        }
           
        else
            return 0;
            
    }


}
