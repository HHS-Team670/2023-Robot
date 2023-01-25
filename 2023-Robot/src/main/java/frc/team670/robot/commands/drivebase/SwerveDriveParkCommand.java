package frc.team670.robot.commands.drivebase;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.robot.subsystems.DriveBase;


// CommandBase or InstantCommand? InstantCommand works just fine
public class SwerveDriveParkCommand extends InstantCommand implements MustangCommand {
 
    private SwerveDrive driveBase;

    private SwerveModuleState[] states; //0 is front left, 1 is front right, 2 is back left, 3 is back right

    public SwerveDriveParkCommand(SwerveDrive driveBase) {
        this.driveBase = driveBase;
    }

    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {

        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        // the drive base requires green so it is just put as that for now
        healthRequirements.put(driveBase, HealthState.GREEN);
        return healthRequirements;

        // return null;
    }

    public void initialize() {
        states = new SwerveModuleState[4];
        // needs the 0.1 or else it won't even rotate the wheels
        states[0] = new SwerveModuleState(0.1, new Rotation2d(Math.PI/4)); // front right
        states[1] = new SwerveModuleState(0.1, new Rotation2d(-Math.PI/4)); // front left
        states[2] = new SwerveModuleState(0.1, new Rotation2d(-Math.PI/4)); // back left
        states[3] = new SwerveModuleState(0.1, new Rotation2d(Math.PI/4)); // back right
    }


    public void execute() {
        driveBase.setModuleStates(states);
    }

    // left empty
    public void end(boolean interrupted) {

    }

}
