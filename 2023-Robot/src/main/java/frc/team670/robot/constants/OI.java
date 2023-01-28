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
import frc.team670.robot.commands.ClawEject;
import frc.team670.robot.commands.ClawIdle;
import frc.team670.robot.subsystems.Claw;

public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);

    // Buttons
    private static JoystickButton clawSuck = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton clawEject = new JoystickButton(driverController, XboxButtons.B);
    private static JoystickButton clawIdle = new JoystickButton(driverController, XboxButtons.A);


    public static MustangController getDriverController() {
        return driverController;
    }

    @Override
    public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
        Claw claw = (Claw) subsystemBases[0];

        clawSuck.onTrue(new ClawIntake(claw)); 
        clawEject.onTrue(new ClawEject(claw));
        clawIdle.onTrue(new ClawIdle(claw));
    }
}