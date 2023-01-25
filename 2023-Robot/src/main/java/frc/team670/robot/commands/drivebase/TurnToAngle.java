package frc.team670.robot.commands.drivebase;

import java.util.HashMap;
import java.util.Map;

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

// testing
// https://github.com/Frc5572/FRC2022/blob/main/src/main/java/frc/robot/commands/TurnToAngle.java
// rotate to angle command
public class TurnToAngle extends CommandBase implements MustangCommand {

    private SwerveDrive swerve;
    private boolean isRelative;
    private double goal;
    private HolonomicDriveController holonomicDriveController;
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();
    private MustangController controller;

    protected Map<MustangSubsystemBase, HealthState> healthReqs;

    public TurnToAngle(SwerveDrive swerve, double angle, boolean isRelative, MustangController controller) {
        this.swerve = swerve;
        this.goal = angle;
        this.isRelative = isRelative;
        this.controller = controller;

        PIDController xcontroller = new PIDController(0, 0, 0);
        PIDController ycontroller = new PIDController(0, 0, 0);
        ProfiledPIDController thetacontroller = new ProfiledPIDController(4, 0, 1,  // not tuned yet
                new Constraints(RobotConstants.kMaxAngularSpeedRadiansPerSecond,
                RobotConstants.kMaxAngularSpeedRadiansPerSecondSquared));
        holonomicDriveController = new HolonomicDriveController(xcontroller, ycontroller, thetacontroller);
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(0.5)));

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
        if (isRelative) {
            targetPose2d = new Pose2d(startPos.getTranslation(),
                    startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
        } else {
            targetPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
        }
    }

    @Override
    public void execute() {
        Pose2d currPose2d = swerve.getPose();
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d, targetPose2d, 0,
                targetPose2d.getRotation());
        SwerveModuleState[] swerveModuleStates = swerve.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
        // System.out.println(targetPose2d.relativeTo(currPose2d));
    }

    @Override
    public void end(boolean interrupt) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] swerveModuleStates = swerve.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
    }

    @Override
    public boolean isFinished() {
        return controller.getBButtonPressed() || holonomicDriveController.atReference();
        // return false;
    }

}