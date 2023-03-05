package frc.team670.robot.commands.vision;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import javax.print.attribute.HashAttributeSet;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

/**
 * IsLockedOn
 */
public class IsLockedOn extends CommandBase implements MustangCommand {

    private DriveBase driveBase;
    private Pose2d goalPose;
    private STATUS turnStatus, strafeStatus, moveStatus;

    private enum STATUS {
        MOVE_UP,
        MOVE_DOWN,
        MOVE_LEFT,
        MOVE_RIGHT,
        TURN_CLOCK,
        TURN_COUNTERCLOCK,
        OKAY
    }

    public IsLockedOn(DriveBase driveBase, Pose2d goalPose) {
        this.driveBase = driveBase;
        this.goalPose = goalPose;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    @Override
    public void execute() {
        updatePoseAlignment();
        SmartDashboard.putStringArray("align", new String[] {moveStatus.name(), strafeStatus.name(), turnStatus.name()});
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("isLockedOn", true);
    }

    private void updatePoseAlignment() {
        Pose2d currentPose = driveBase.getPose();
        double driverDX = allianceAdjustment(goalPose.getX() - currentPose.getX());
        double driverDY = allianceAdjustment(goalPose.getY() - currentPose.getY());
        double dRot = goalPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
        
        if (driverDX > RobotConstants.LOCKED_ON_ERROR_X) {
            moveStatus = STATUS.MOVE_UP;
        } else if (driverDX < RobotConstants.LOCKED_ON_ERROR_X) {
            moveStatus = STATUS.MOVE_DOWN;
        } else {
            moveStatus = STATUS.OKAY;
        }
        
        if (driverDY > RobotConstants.LOCKED_ON_ERROR_Y) {
            strafeStatus = STATUS.MOVE_RIGHT;
        } else if (driverDY < RobotConstants.LOCKED_ON_ERROR_Y) {
            strafeStatus = STATUS.MOVE_LEFT;
        } else {
            strafeStatus = STATUS.OKAY;
        }
        
        if (dRot > RobotConstants.LOCKED_ON_ERROR_DEGREES) {
            turnStatus = STATUS.TURN_CLOCK;
        } else if (dRot < RobotConstants.LOCKED_ON_ERROR_DEGREES) {
            turnStatus = STATUS.TURN_COUNTERCLOCK;
        } else {
            turnStatus = STATUS.OKAY;
        }
    }

    private double allianceAdjustment(double n) {
        return DriverStation.getAlliance() == Alliance.Blue ? n : -1 * n;
    }

}
