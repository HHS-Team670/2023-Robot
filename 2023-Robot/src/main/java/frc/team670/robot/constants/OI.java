package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.leds.SetIntakeCone;
import frc.team670.robot.commands.leds.SetIntakeCube;
import frc.team670.robot.subsystems.LED;


public final class OI {

    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);


  
    private static JoystickButton coneSuck = new JoystickButton(operatorController, XboxButtons.Y);
    private static JoystickButton cubeSuck = new JoystickButton(operatorController, XboxButtons.A);

    
    public static MustangController getDriverController() {
        return driverController;
    }

    public static MustangController getOperatorController() {
        return operatorController;
    }

    public static void configureButtonBindings() {
        LED led = LED.getInstance();
    
        cubeSuck.onTrue(new SetIntakeCube(led));
        coneSuck.onTrue(new SetIntakeCone(led));
    }
}
