package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.RobotConstants;

/**
 * MoveToPose
 */
public class MoveToPosePID extends CommandBase implements MustangCommand {
    private SwerveDrive swerve;
    private boolean isRelative;
    private HolonomicDriveController holonomicDriveController;
    private Pose2d startPose;
    private Pose2d targetPose;
    // private MustangController controller;
    private double x, y;

    protected Map<MustangSubsystemBase, HealthState> healthReqs;


    // public MoveToPosePID(SwerveDrive swerve, Pose2d pose, boolean isRelative, MustangController
    // controller) {
    public MoveToPosePID(SwerveDrive swerve, Pose2d pose, boolean isRelative) {
        this.swerve = swerve;
        this.startPose = new Pose2d();
        this.targetPose = new Pose2d();
        this.isRelative = isRelative;
        // this.controller = controller;
        this.x = pose.getX();
        this.y = pose.getY();

        PIDController xController = new PIDController(0.5, 0, 0);
        PIDController yController = new PIDController(0.5, 0, 0);
        ProfiledPIDController thetacontroller = new ProfiledPIDController(4, 0, 1, // not tuned yet
                new Constraints(RobotConstants.kMaxAngularSpeedRadiansPerSecond,
                        RobotConstants.kMaxAngularSpeedRadiansPerSecondSquared));
        holonomicDriveController =
                new HolonomicDriveController(xController, yController, thetacontroller);
        holonomicDriveController.setTolerance(new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.5)));

        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(swerve, HealthState.GREEN);
    }

    // public MoveToPosePID(SwerveDrive swerve, double x, double y, boolean isRelative,
    // MustangController controller) {
    public MoveToPosePID(SwerveDrive swerve, double x, double y, boolean isRelative) {
        this.swerve = swerve;
        this.startPose = new Pose2d();
        this.targetPose = new Pose2d();
        this.isRelative = isRelative;
        // this.controller = controller;
        this.x = x;
        this.y = y;

        PIDController xController = new PIDController(0.5, 0, 0);
        PIDController yController = new PIDController(0.5, 0, 0);
        ProfiledPIDController thetacontroller = new ProfiledPIDController(4, 0, 1, // not tuned yet
                new Constraints(RobotConstants.kMaxAngularSpeedRadiansPerSecond,
                        RobotConstants.kMaxAngularSpeedRadiansPerSecondSquared));
        holonomicDriveController =
                new HolonomicDriveController(xController, yController, thetacontroller);
        holonomicDriveController.setTolerance(new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(0.5)));

        this.healthReqs = new HashMap<MustangSubsystemBase, HealthState>();
        this.healthReqs.put(swerve, HealthState.GREEN);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthReqs;
    }

    @Override
    public void initialize() {
        startPose = swerve.getOdometerPose();
        if (isRelative) {
            targetPose = startPose.plus(new Transform2d(new Translation2d(x, y), new Rotation2d()));
        } else {
            targetPose = new Pose2d(x, y, new Rotation2d());
        }
    }

    @Override
    public void execute() {
        Pose2d currPose2d = swerve.getOdometerPose();
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
                targetPose, 0, targetPose.getRotation());
        SwerveModuleState[] swerveModuleStates =
                swerve.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
    }

    @Override
    public boolean isFinished() {
        // return controller.getBButtonPressed() || holonomicDriveController.atReference();
        return holonomicDriveController.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        // stop
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] swerveModuleStates =
                swerve.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
    }
}
