package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.subsystems.DriveBase;

public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);

    // Buttons
    private static JoystickButton zeroGyro = new JoystickButton(driverController, XboxButtons.X);

    public static MustangController getDriverController() {
        return driverController;
    }

    @Override
    public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
        DriveBase driveBase = (DriveBase) subsystemBases[0];

        driveBase.initDefaultCommand();

        zeroGyro.onTrue(new SetSwerveForwardDirection(driveBase)); // deprecated Button.whenPressed(), used Trigger.onTrue()
    }
    
}
