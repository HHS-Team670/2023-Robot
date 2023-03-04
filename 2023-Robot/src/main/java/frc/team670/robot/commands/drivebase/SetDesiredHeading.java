package frc.team670.robot.commands.drivebase;

import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;

public class SetDesiredHeading extends InstantCommand implements MustangCommand {
    SwerveDrive driveBase;
    int assignedButton;

    public SetDesiredHeading(SwerveDrive driveBase, int assignedButton) {
        this.driveBase = driveBase;
        this.assignedButton = assignedButton;


    }
    
    @Override
    public void initialize() {
        switch (assignedButton) {
            case XboxButtons.Y:
            driveBase.setDesiredHeading(new Rotation2d(0));
            break;
            case XboxButtons.X:
            driveBase.setDesiredHeading(new Rotation2d(Math.PI / 2));
            break;
            case XboxButtons.A:
            driveBase.setDesiredHeading(new Rotation2d(Math.PI));
            break;
            case XboxButtons.B:
            driveBase.setDesiredHeading(new Rotation2d(3 * Math.PI / 2));
            break;
        }
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
