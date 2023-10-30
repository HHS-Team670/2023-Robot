package frc.team670.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team670.mustanglib.commands.drive.teleop.swerve.SetSwerveForwardDirection;
import frc.team670.mustanglib.commands.drive.teleop.swerve.XboxSwerveDrive;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.arm.ManualMoveElbow;
import frc.team670.robot.commands.arm.ManualMoveWrist;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.arm.ResetArmFromAbsolute;
import frc.team670.robot.commands.arm.ResetArmOffset;
import frc.team670.robot.commands.claw.ClawIdle;
import frc.team670.robot.commands.claw.ClawInstantIntake;
import frc.team670.robot.commands.cubeintake.CubeIntakeToggle;
import frc.team670.robot.commands.cubeintake.ToggleCubeIntakeDeployer;
import frc.team670.robot.commands.drivebase.MoveToCone;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
import frc.team670.robot.commands.leds.SetIntakeCube;
import frc.team670.robot.commands.leds.Blinking;
import frc.team670.robot.commands.leds.Rainbow;
import frc.team670.robot.commands.leds.SetIntakeCone;
import frc.team670.robot.commands.routines.DualEject;
import frc.team670.robot.commands.routines.DualIntake;
import frc.team670.robot.commands.routines.DualIdle;
import frc.team670.robot.commands.routines.EjectAndStow;
import frc.team670.robot.commands.vision.AutoAlign;
import frc.team670.robot.commands.vision.AutoAlignToSubstation;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import frc.team670.robot.subsystems.drivebase.DriveBase;
import frc.team670.robot.subsystems.CubeIntake;


public final class OI {

    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);
    private static MustangController backupController = new MustangController(2);

    // LED commands
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
       
        cubeSuck.onTrue(new Rainbow(led));
        coneSuck.onTrue(new Blinking(led));
        // turnToCone.onTrue(new MoveToCone(driveBase, driverController));

        // autoLevel.onTrue(new NonPidAutoLevel(driveBase, true));
    }
}
