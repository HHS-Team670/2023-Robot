package frc.team670.robot.commands.drivebase;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.subsystems.DriveBase;

public class Creep extends CommandBase implements MustangCommand {
    public static final double CREEP_SPEED = 0.3;
    private DriveBase driveBase;
    

    public Creep(DriveBase driveBase) {
        this.driveBase = driveBase;
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        Rotation2d holomnicRotation = driveBase.getGyroscopeRotation();
        double vx = CREEP_SPEED * Math.cos(holomnicRotation.getRadians());
        double vy = CREEP_SPEED * Math.sin(holomnicRotation.getRadians());

        driveBase.drive(new ChassisSpeeds(vx, vy, 0));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
