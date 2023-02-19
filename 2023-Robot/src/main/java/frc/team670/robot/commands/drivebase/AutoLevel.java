package frc.team670.robot.commands.drivebase;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.PIDConstantSet;
import frc.team670.robot.subsystems.DriveBase;

public class AutoLevel extends CommandBase implements MustangCommand {
    DriveBase driveBase;
    double target = 0;
    int counter = 0;

    PIDController controller = new PIDController(0.05, 0, 0.015); //p: 0.03, d: 0.025

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
        SmartDashboard.putBoolean("level", false); //TODO: will change this line after this command is finished
        double pitch = driveBase.getPitch();
        SmartDashboard.putNumber("pitch", pitch);

        // Run while facing positive X direction
        //double adjustedSpeed = MathUtil.clamp((target - pitch) * kp, -1, 1); //This may need to be PLUS (pitch-prevPitch)*kD, rather than minus. Please test!
        if (counter % 10 == 0) {
            double adjustedSpeed = controller.calculate(pitch, target);
            SmartDashboard.putNumber("speed", adjustedSpeed);
            
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(adjustedSpeed, 0.0, 0.0);
            SwerveModuleState[] states = driveBase.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
            driveBase.setModuleStates(states);
        }
        counter++;
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("level", true);
        return false;
    }
}
