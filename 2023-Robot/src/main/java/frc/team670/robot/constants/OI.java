package frc.team670.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.commands.vision.AutoAlign;
import frc.team670.robot.commands.vision.IsLockedOn;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;

public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);

    // Buttons
    private static JoystickButton zeroGyro = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton moveToTarget = new JoystickButton(driverController, XboxButtons.Y);


    public static MustangController getDriverController() {
        return driverController;
    }

    @Override
    public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
        DriveBase driveBase = (DriveBase) subsystemBases[0];
        Vision vision = (Vision) subsystemBases[1];

        driveBase.initDefaultCommand();
        vision.initDefaultCommand(new IsLockedOn(driveBase, vision));

        zeroGyro.onTrue(new SetSwerveForwardDirection(driveBase)); // deprecated
                                                                   // Button.whenPressed(), used
                                                                   // Trigger.onTrue()
        moveToTarget.onTrue(new AutoAlign(vision, driveBase));

        // move.onTrue(new MoveToPose(driveBase, new Pose2d(1, 1, new Rotation2d()), true));
    }

}
