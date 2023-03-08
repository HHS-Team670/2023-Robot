package frc.team670.robot.commands.drivebase;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class NonPidAutoLevel extends CommandBase implements MustangCommand {
    DriveBase driveBase;
    double target = 0;
    double error = 2.5; // 2.5 degrees is allowable
    double pitch;
    double previousPitch;
    boolean hasGoneUp = false;
    boolean fromDriverSide = false;


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

        pitch = Math.abs(driveBase.getPitch());
        previousPitch = Math.abs(driveBase.getPitch()); // just to ensure we are going forward
        this.hasGoneUp = false;
    }

    @Override
    public void execute() {
        previousPitch = pitch;
        //SmartDashboard.putNumber("previousPitch", previousPitch);
        pitch = Math.abs(driveBase.getPitch());
        // SmartDashboard.putNumber("pitch", pitch);

        // SmartDashboard.putNumber("non pid pose x", driveBase.getPose().getX());
        // SmartDashboard.putNumber("non pid pose y", driveBase.getPose().getY());


        if (pitch > 5) {
            hasGoneUp = true;
        }

        if ((previousPitch - pitch) < 0.5) { //While going up the ramp...
            ChassisSpeeds chassisSpeeds;
            if(fromDriverSide) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.75, 0, 0, driveBase.getGyroscopeRotation());
            } else {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.75, 0, 0, driveBase.getGyroscopeRotation());
            }
            SwerveModuleState[] states = driveBase.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
            driveBase.setModuleStates(states);
        } else {
            SwerveModuleState[] states = new SwerveModuleState[4];
                states[0] = new SwerveModuleState(0.01, new Rotation2d(Math.PI/4)); 
                states[1] = new SwerveModuleState(0.01, new Rotation2d(-Math.PI/4));
                states[2] = new SwerveModuleState(0.01, new Rotation2d(-Math.PI/4));
                states[3] = new SwerveModuleState(0.01, new Rotation2d(Math.PI/4));
                driveBase.setModuleStates(states);
        }
    }

    @Override
    public boolean isFinished() {
        if (driveBase.getPitch() > (target - error) && driveBase.getPitch() < (target + error) && hasGoneUp) {
            return true;
        }
        // SmartDashboard.putBoolean("level", level);
        return false;
    }
}
