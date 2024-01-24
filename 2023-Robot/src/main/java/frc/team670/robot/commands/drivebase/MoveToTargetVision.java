package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.drivebase.DriveBase;
/**
 * MoveToPose - moves to specified pose. Cancels when button is released.
 */
public class MoveToTargetVision extends CommandBase implements MustangCommand {
    private DriveBase driveBase;
    // private final Pose2d endPose;
    private PhotonCamera[] cameras;
    // private boolean backOut = false;
    protected Map<MustangSubsystemBase, HealthState> healthReqs;
    private MustangController mController;
    private final static double kSensitivity=0.5;
    private double prevTimestamp = 0;


    public MoveToTargetVision(DriveBase driveBase,MustangController mController){
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
        cameras = driveBase.getPoseEstimator().getVision().getCameras();
        
        
    }
    @Override
    public void execute(){
        // SmartDashboard.putNumber("Theta Velocit",kSensitivity*angleToTarget());
        driveBase.setmDesiredHeading(new Rotation2d(driveBase.getPose().getRotation().getRadians() + angleToTarget()));    
    }

    public boolean isFinished(){
        return !mController.getRightBumper();
        //return mController.getRightTriggerAxis() > 0.7;
    }



    public boolean hasTarget(){
       int targets = 0;
       for (PhotonCamera c : cameras){
            var result = c.getLatestResult();
            targets += result.getTargets().size();
       }
       return targets > 0;
    }
    //
    public double angleToTarget(){
        double distance = 0;
        double bestDistance = 1000;
        PhotonTrackedTarget bestTarget = null; 
        double bestTargetLatency = 0;
        for (PhotonCamera c : cameras){
            try{
                var result = c.getLatestResult();
                if(prevTimestamp < result.getTimestampSeconds()){
                    continue;
                } 
                if(result.hasTargets()){
                    for (PhotonTrackedTarget target : result.getTargets()) {
                        var transform = target.getBestCameraToTarget();
                        distance = Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2) + Math.pow(transform.getZ(), 2) + Math.pow((Math.abs(Math.PI/2 - transform.getRotation().getZ())), 2));
                        if(distance < bestDistance) {
                            bestTarget = target;
                            bestDistance = distance;
                            bestTargetLatency = result.getLatencyMillis()/1000;
                            prevTimestamp = result.getTimestampSeconds();
                        }
                    } 
                }
            }
            catch(java.lang.ArrayIndexOutOfBoundsException e){
                
            }
           
        }
        
        if(bestTarget != null){
            double factor = calcLatencyFactor(bestTarget, bestTargetLatency);
            SmartDashboard.putNumber("A Latency Factor", factor);
                        double result = -(bestTarget.getYaw() + factor) * Math.PI / 180;

            SmartDashboard.putNumber("A Turning", result);
            return result;
        }
       return 0;
    }
    public double calcLatencyFactor(PhotonTrackedTarget bestTarget, double bestTargetLatency){
        //Rotation2d heading = driveBase.getGyroscopeRotation();
        double distanceX = bestTarget.getBestCameraToTarget().getX();
        return bestTargetLatency* driveBase.getChassisSpeeds().vyMetersPerSecond * (100/distanceX);
    }
    
}

