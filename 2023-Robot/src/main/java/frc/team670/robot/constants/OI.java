package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.vision.AutoAlign;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);

    // Driver buttons
    private static JoystickButton zeroGyro = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton moveToTarget = new JoystickButton(driverController, XboxButtons.Y);

    //private static JoystickButton move = new JoystickButton(driverController, XboxButtons.Y);

    // Operator buttons
    private static POVButton hybrid = new POVButton(operatorController, 180);
    private static POVButton scoreMid = new POVButton(operatorController, 90);
    private static POVButton scoreHigh = new POVButton(operatorController, 0);


    public static MustangController getDriverController() {
        return driverController;
    }

    public static MustangController getOperatorController() {
        return operatorController;
    }

    @Override
    public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
        DriveBase driveBase = (DriveBase) subsystemBases[0];
        Vision vision = (Vision) subsystemBases[1];
        Arm arm = (Arm) subsystemBases[2];

        driveBase.initDefaultCommand();

        zeroGyro.onTrue(new SetSwerveForwardDirection(driveBase)); // deprecated
                                                                   // Button.whenPressed(), used
                                                                   // Trigger.onTrue()
        moveToTarget.onTrue(new AutoAlign(vision, driveBase));
        // move.onTrue(new MoveToPose(driveBase, new Pose2d(1, 1, new Rotation2d()), true));

        // //arm movement commands
        hybrid.onTrue(new MoveToTarget(arm, ArmState.HYBRID));
        scoreMid.onTrue(new MoveToTarget(arm, ArmState.SCORE_MID));
        scoreHigh.onTrue(new MoveToTarget(arm, ArmState.SCORE_HIGH));

        hybrid.onFalse(new MoveToTarget(arm, ArmState.STOWED));
        scoreMid.onFalse(new MoveToTarget(arm, ArmState.STOWED));
        scoreHigh.onFalse(new MoveToTarget(arm, ArmState.STOWED));

    }

}
