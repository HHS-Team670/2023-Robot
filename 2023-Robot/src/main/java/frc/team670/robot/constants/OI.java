package frc.team670.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);

    // Buttons
    private static JoystickButton zeroGyro = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton move = new JoystickButton(driverController, XboxButtons.Y);
    private static JoystickButton highCone = new JoystickButton(driverController, XboxButtons.B);
    private static JoystickButton stowButton = new JoystickButton(driverController, XboxButtons.A);
    

    public static MustangController getDriverController() {
        return driverController;
    }

    @Override
    public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
        DriveBase driveBase = (DriveBase) subsystemBases[0];
        Arm arm = (Arm) subsystemBases[1];

        driveBase.initDefaultCommand();

        zeroGyro.onTrue(new SetSwerveForwardDirection(driveBase)); // deprecated
                                                                   // Button.whenPressed(), used
                                                                   // Trigger.onTrue()
        move.onTrue(new MoveToPose(driveBase, 1, 1, false));
        highCone.onTrue(new MoveToTarget(arm, ArmState.SCORE_HIGH));
        stowButton.onTrue(new MoveToTarget(arm, ArmState.HOPPER));
    }

}
