package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.DPadState;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.ClawIntake;
import frc.team670.robot.subsystems.Claw;

public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);

    // Buttons
    private static JoystickButton clawjoystick = new JoystickButton(driverController, XboxButtons.X);


    public static MustangController getDriverController() {
        return driverController;
    }

    @Override
    public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
        Claw claw = (Claw) subsystemBases[0];

        clawjoystick.onTrue(new ClawIntake(claw)); 
    }
}