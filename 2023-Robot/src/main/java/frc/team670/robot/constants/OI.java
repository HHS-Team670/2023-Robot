package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team670.mustanglib.commands.MustangScheduler;
//import frc.team670.mustanglib.commands.drive.teleop.ResetArmFromAbsolute;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.arm.ManualMoveElbow;
import frc.team670.robot.commands.arm.ManualMoveShoulder;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawEject;
import frc.team670.robot.commands.claw.ClawIdle;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.commands.vision.AutoAlign;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class OI extends OIBase {
    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);

    // Driver buttons
    private static JoystickButton zeroGyro = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton zeroArm = new JoystickButton(operatorController, XboxButtons.X);
    private static JoystickButton moveToTarget = new JoystickButton(driverController, XboxButtons.Y);

    // private static JoystickButton move = new JoystickButton(driverController,
    // XboxButtons.Y);

    // Operator buttons
    private static POVButton backward = new POVButton(operatorController, 180);
    private static POVButton scoreMidR = new POVButton(operatorController, 90);
    private static POVButton scoreMidL = new POVButton(operatorController, 270);
    private static POVButton scoreHigh = new POVButton(operatorController, 0);
    private static JoystickButton stow = new JoystickButton(operatorController, XboxButtons.B);
    private static JoystickButton manualElbowControl = new JoystickButton(operatorController,
            XboxButtons.RIGHT_JOYSTICK_BUTTON);
    private static JoystickButton manualShoulderControl = new JoystickButton(operatorController,
            XboxButtons.LEFT_JOYSTICK_BUTTON);

    private static JoystickButton clawSuck = new JoystickButton(operatorController, XboxButtons.RIGHT_BUMPER);
    private static JoystickButton clawEject = new JoystickButton(operatorController, XboxButtons.LEFT_BUMPER);
    private static JoystickButton clawIdle = new JoystickButton(operatorController, XboxButtons.Y);

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
        Claw claw = (Claw) subsystemBases[3];

        driveBase.initDefaultCommand();

        //zeroGyro.onTrue(new SetSwerveForwardDirection(driveBase));
        // zeroArm.onTrue(new ResetArmFromAbsolute(arm)); DO NOT USE!!! Critial errors not yet fixed
        moveToTarget.onTrue(new AutoAlign(vision, driveBase));
        // move.onTrue(new MoveToPose(driveBase, new Pose2d(1, 1, new Rotation2d()),
        // true));

        // //arm movement commands
        backward.onTrue(new MoveToTarget(arm, ArmState.BACKWARD_GROUND));
        scoreMidR.onTrue(new MoveToTarget(arm, ArmState.SCORE_MID));
        scoreMidL.onTrue(new MoveToTarget(arm, ArmState.SCORE_MID));
        scoreHigh.onTrue(new MoveToTarget(arm, ArmState.SCORE_HIGH));
        stow.onTrue(new MoveToTarget(arm, ArmState.STOWED));
        manualShoulderControl.onTrue(new ManualMoveShoulder(arm, operatorController));

        manualElbowControl.onTrue(new ManualMoveElbow(arm, operatorController));
        // Claw control commands
        clawSuck.onTrue(new ClawIntake(claw));
        clawEject.onTrue(new ClawEject(claw));
        clawIdle.onTrue(new ClawIdle(claw));

    }

}
