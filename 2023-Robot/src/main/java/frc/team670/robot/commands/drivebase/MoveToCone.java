package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.RobotConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// testing
// https://github.com/Frc5572/FRC2022/blob/main/src/main/java/frc/robot/commands/TurnToAngle.java
// rotate to angle command
public class MoveToCone extends CommandBase implements MustangCommand {

    private SwerveDrive swerve;

    private HolonomicDriveController holonomicDriveController;
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();
    private MustangController controller;
    private PhotonCamera colorCam;

    protected Map<MustangSubsystemBase, HealthState> healthReqs;

    public MoveToCone(SwerveDrive swerve, MustangController mController) {
        this.swerve = swerve;

        this.controller = mController;

        PIDController xcontroller = new PIDController(0, 0, 0);
        PIDController ycontroller = new PIDController(0, 0, 0);
        ProfiledPIDController thetacontroller = new ProfiledPIDController(4, 0, 1, // not tuned yet
                new Constraints(RobotConstants.DriveBase.kMaxAngularSpeedRadiansPerSecond,
                        RobotConstants.DriveBase.kMaxAngularAccelerationRadiansPerSecondSquared));

        holonomicDriveController =
                new HolonomicDriveController(xcontroller, ycontroller, thetacontroller);
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(3)));

        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(swerve, HealthState.GREEN);
    }

   

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        startPos = swerve.getPose();
        colorCam = swerve.getPoseEstimator().getVision().getCameras()[1];
        targetPose2d = new Pose2d(startPos.getTranslation(),startPos.getRotation().rotateBy(Rotation2d.fromDegrees(angleToCone())));
       
    }

    @Override
    public void execute() {
        Pose2d currPose2d = swerve.getPose();
        targetPose2d = new Pose2d(startPos.getTranslation(),startPos.getRotation().rotateBy(Rotation2d.fromDegrees(angleToCone())));
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
                targetPose2d, 0, targetPose2d.getRotation());
        SwerveModuleState[] swerveModuleStates =
                swerve.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
        // System.out.println(targetPose2d.relativeTo(currPose2d));
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putNumber("current rotation error", targetPose2d.getRotation().minus(swerve.getPose().getRotation()).getDegrees());
        SmartDashboard.putBoolean("isAtReference", holonomicDriveController.atReference());
        return controller == null ? false : !controller.getRightBumper();
        // return holonomicDriveController.atReference();
        // return false;
    }

    @Override
    public void end(boolean interrupt) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] swerveModuleStates =
                swerve.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
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
            return -result.getTargets().get(0).getYaw();
        }
           
        else
            return 0;
            
    }

}
