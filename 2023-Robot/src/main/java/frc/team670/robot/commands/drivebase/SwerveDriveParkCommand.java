// package frc.team670.robot.commands.drivebase;

// import java.util.HashMap;
// import java.util.Map;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
// import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

// // CommandBase or InstantCommand? InstantCommand works just fine
// public class SwerveDriveParkCommand extends InstantCommand implements MustangCommand {

//     private SwerveDrive driveBase;

//     public SwerveDriveParkCommand(SwerveDrive driveBase) {
//         this.driveBase = driveBase;
//     }

//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {

//         Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
//         // the drive base requires green so it is just put as that for now
//         healthRequirements.put(driveBase, HealthState.GREEN);
//         return healthRequirements;

//         // return null;
//     }

//     public void initialize() {
//         driveBase.park();
//     }

// }
