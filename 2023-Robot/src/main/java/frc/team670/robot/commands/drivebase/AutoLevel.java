package frc.team670.robot.commands.drivebase;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class AutoLevel extends CommandBase implements MustangCommand {
    DriveBase driveBase;
    double target = 0;
    double kp = 0.03;

    public AutoLevel(DriveBase driveBase) {
        this.driveBase = driveBase;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double pitch = driveBase.getPitch();
        SmartDashboard.putNumber("pitch", pitch);

        double adjustedSpeed = MathUtil.clamp((target - pitch) * kp, -1, 1);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(adjustedSpeed, 0.0, 0.0);
        SwerveModuleState[] states = driveBase.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
        driveBase.setModuleStates(states);
        
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("level", true);
        return false;
    }
}
