package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.RobotConstantsBase;
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
    private double targetDistX;
    private double targetDistY;
    private double[] lastChange;



    public MoveToTargetVision(DriveBase driveBase,MustangController mController, double targetDistX, double targetDistY){
        this.driveBase = driveBase;
        this.mController=mController;
        this.targetDistX = targetDistX;
        this.targetDistY = targetDistY;
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
        SmartDashboard.putNumber("A value", driveBase.getGyroscopeRotation().getRadians());
        SmartDashboard.putNumber("A controller value", mController.getRightStickX() + mController.getRightStickY());
        double[] changes = moveToTarget();

        SmartDashboard.putNumber("A dX", changes[0]);
        SmartDashboard.putNumber("A dY", changes[1]);
        SmartDashboard.putNumber("A dTheta", changes[2]);

        double xVel = RobotConstantsBase.SwerveDriveBase.xController.calculate(changes[0]);
        double yVel = RobotConstantsBase.SwerveDriveBase.yController.calculate(changes[1]);
        double thetaVel = RobotConstantsBase.SwerveDriveBase.thetaController.calculate(changes[2]);
        driveBase.drive((ChassisSpeeds.fromFieldRelativeSpeeds(-xVel, -yVel, thetaVel,
                driveBase.getGyroscopeRotation())));
        
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
    public double[] moveToTarget(){
        double distance = 0;
        double bestDistance = 1000;
        PhotonTrackedTarget bestTarget = null; 
        double bestTargetLatency = 0;
        PhotonCamera c = null;
        for (int i = 0; i < cameras.length; i++){
            try{
                c = cameras[i];
                var result = c.getLatestResult();
                // if(prevTimestamp >= result.getTimestampSeconds()){
                //     continue;
                // } 
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
            //Turning
            double factor = calcLatencyFactor(bestTarget, bestTargetLatency);
            SmartDashboard.putNumber("A Latency Factor", factor);
            double dTheta = -(bestTarget.getYaw() + factor) * Math.PI / 180;
            SmartDashboard.putNumber("A Turning", dTheta);

            //Moving
            var transform = bestTarget.getBestCameraToTarget();
            double dX = (targetDistX - transform.getX());
            double dY = (targetDistY - transform.getY());

            lastChange = new double[]{dX, dY, dTheta};

            return lastChange;
        }

        if(lastChange != null)
            return lastChange;

       return new double[3];
    }
    public double calcLatencyFactor(PhotonTrackedTarget bestTarget, double bestTargetLatency){
        //Rotation2d heading = driveBase.getGyroscopeRotation();
        double distanceX = bestTarget.getBestCameraToTarget().getX();
        return bestTargetLatency* driveBase.getChassisSpeeds().vyMetersPerSecond * (100/distanceX);
    }
    
    public double adjustForDeceleration(){
        
        return 0;
    }
}

