package frc.team670.robot.commands.drivebase;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

public class SetDesiredHeading extends InstantCommand implements MustangCommand {
    SwerveDrive driveBase;
    Rotation2d desiredHeading;

    public SetDesiredHeading(SwerveDrive driveBase, Rotation2d desiredHeading) {
        this.driveBase = driveBase;
        this.desiredHeading = desiredHeading;
    }
    
    @Override
    public void initialize() {
        driveBase.setDesiredHeading(desiredHeading);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
