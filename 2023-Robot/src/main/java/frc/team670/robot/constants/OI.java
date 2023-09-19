package frc.team670.robot.constants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.leds.ChangeColor;
import frc.team670.robot.subsystems.LED;


public final class OI {

    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);

    // LED commands
    private static JoystickButton increaseHex = new JoystickButton(operatorController, XboxButtons.Y);
    private static JoystickButton decreaseHex = new JoystickButton(operatorController, XboxButtons.A);

    
    public static MustangController getDriverController() {
        return driverController;
    }

    public static MustangController getOperatorController() {
        return operatorController;
    }

    public static void configureButtonBindings() {
        LED led = LED.getInstance();

        increaseHex.onTrue(new ChangeColor(led, true));
        decreaseHex.onTrue(new ChangeColor(led, false));
    }
}
