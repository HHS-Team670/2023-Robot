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
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import frc.team670.robot.subsystems.drivebase.DriveBase;

public class AutoLevelShort extends CommandBase implements MustangCommand {
    DriveBase driveBase;
    double target = 0;
    double error = 2.5; // 2.5 degrees is allowable
    double pitch;
    boolean fromDriverSide = false;
    int counter;



    public AutoLevelShort(DriveBase driveBase, boolean fromDriverSide) {
        this.driveBase = driveBase;
        this.fromDriverSide = fromDriverSide;
        SmartDashboard.putNumber("backtracking iterations", 50);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    @Override
    public void initialize() {
        counter = 0;
        pitch = Math.abs(driveBase.getPitch()+ driveBase.getRoll());
    }

    @Override
    public void execute() {
        pitch = Math.abs(driveBase.getPitch() + driveBase.getRoll());
        SmartDashboard.putNumber("pitch", pitch);
        // SmartDashboard.putBoolean("hasGoneUp", hasGoneUp);
        // SmartDashboard.putBoolean("hasTippedOver", hasTippedOver);

        // SmartDashboard.putNumber("non pid pose x", driveBase.getPose().getX());
        // SmartDashboard.putNumber("non pid pose y", driveBase.getPose().getY());


        ChassisSpeeds chassisSpeeds;

        
        if (fromDriverSide) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-0.45, 0, 0, driveBase.getGyroscopeRotation());
        } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.45, 0, 0, driveBase.getGyroscopeRotation());
        }
        counter++;

        SwerveModuleState[] states = driveBase.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
        driveBase.setModuleStates(states);
        
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(pitch) < 2 && counter > 20) { 
            //SmartDashboard.putBoolean("isFinished", true);
            return true;
        }
        // if (driveBase.getPitch() > (target - error) && driveBase.getPitch() < (target + error) && hasGoneUp) {
        //     return true;
        // }
        // SmartDashboard.putBoolean("level", level);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //SmartDashboard.putBoolean("non PID auto level ended", true);
            SwerveModuleState[] states = new SwerveModuleState[4];
                states[0] = new SwerveModuleState(0.01, new Rotation2d(Math.PI/4)); 
                states[1] = new SwerveModuleState(0.01, new Rotation2d(-Math.PI/4));
                states[2] = new SwerveModuleState(0.01, new Rotation2d(-Math.PI/4));
                states[3] = new SwerveModuleState(0.01, new Rotation2d(Math.PI/4));
                driveBase.setModuleStates(states);
    }
}
