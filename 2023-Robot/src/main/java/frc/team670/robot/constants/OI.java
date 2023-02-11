package frc.team670.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.commands.vision.AutoAlign;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.commands.arm.MoveDirectlyToTarget;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);

    // Buttons
    private static JoystickButton zeroGyro = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton moveToTarget = new JoystickButton(driverController, XboxButtons.Y);
    //private static JoystickButton move = new JoystickButton(driverController, XboxButtons.Y);
    private static JoystickButton hopper = new JoystickButton(driverController, XboxButtons.B);
    private static JoystickButton stow = new JoystickButton(driverController, XboxButtons.A);
    private static JoystickButton highCone = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    private static JoystickButton hybrid = new JoystickButton(driverController, XboxButtons.RIGHT_BUMPER);

    public static MustangController getDriverController() {
        return driverController;
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

        move.onTrue(new MoveToPose(driveBase, 1, 1, false));

        stow.onTrue(new MoveToTarget(arm, ArmState.STOWED)); 
        hopper.onTrue(new MoveToTarget(arm, ArmState.HOPPER));
        hybrid.onTrue(new MoveToTarget(arm, ArmState.HYBRID));
        highCone.onTrue(new MoveToTarget(arm, ArmState.SCORE_HIGH));
    }

}
