package frc.team670.robot.commands.drivebase;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class NonPidAutoLevel extends CommandBase implements MustangCommand {

    DriveBase driveBase;
    
    double target = 0;
    double error = 2.5;
    double pitch;
    double previousPitch;
    double multiplier; // related to fromDriverSide

    boolean hasGoneUp = false;
    boolean fromDriverSide = false;
    boolean parked;


    public NonPidAutoLevel(DriveBase driveBase, boolean fromDriverSide) {
        this.driveBase = driveBase;
        this.fromDriverSide = fromDriverSide;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    @Override
    public void initialize() {
        // SmartDashboard.putNumber("init non pid pose x", driveBase.getPose().getX());
        // SmartDashboard.putNumber("init non pid pose y", driveBase.getPose().getY());

        pitch = Math.abs(driveBase.getPitch());
        previousPitch = Math.abs(driveBase.getPitch());
        hasGoneUp = false;
        parked = false;

        if (fromDriverSide) {
            multiplier = 1;
        }
        else {
            multiplier = -1;
        }
    }

    @Override
    public void execute() {
        previousPitch = pitch;
        // SmartDashboard.putNumber("previousPitch", previousPitch);
        pitch = Math.abs(driveBase.getPitch());
        // SmartDashboard.putNumber("pitch", pitch);
        // SmartDashboard.putNumber("non pid pose x", driveBase.getPose().getX());
        // SmartDashboard.putNumber("non pid pose y", driveBase.getPose().getY());


        if (pitch > 5) {
            hasGoneUp = true;
        }

        if ((previousPitch - pitch) < 0.25) { // going up the ramp
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.75 * multiplier, 0, 0, driveBase.getGyroscopeRotation());
            SwerveModuleState[] states = driveBase.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
            driveBase.setModuleStates(states);
        }
        else { // going down now
            parked = true;
            MustangScheduler.getInstance().schedule(new SwerveDriveParkCommand(driveBase), driveBase);
        }
    }

    @Override
    public boolean isFinished() {
        if (driveBase.getPitch() > (target - error) && driveBase.getPitch() < (target + error) && hasGoneUp) {
            return true;
        }
        if (parked) { // parked, but not leveled
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.2 * multiplier, 0, 0, driveBase.getGyroscopeRotation());
            SwerveModuleState[] states = driveBase.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
            driveBase.setModuleStates(states);
        }
        return false;
    }
}
