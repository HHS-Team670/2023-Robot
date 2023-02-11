package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.ClawIntake;
import frc.team670.robot.commands.ClawEject;
import frc.team670.robot.subsystems.Claw;

public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);

    // Buttons
    private static JoystickButton clawSuck = new JoystickButton(operatorController, XboxButtons.A);
    private static JoystickButton clawEject = new JoystickButton(operatorController, XboxButtons.B);


    public static MustangController getDriverController() {
        return driverController;
    }

    public static MustangController getOperatorController() {
        return operatorController;
    }

    @Override
    public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
        Claw claw = (Claw) subsystemBases[0];

        clawSuck.onTrue(new ClawIntake(claw)); 
        clawEject.onTrue(new ClawEject(claw));
    }
}