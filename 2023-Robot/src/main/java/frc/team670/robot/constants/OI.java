package frc.team670.robot.constants;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.arm.ManualMoveElbow;
import frc.team670.robot.commands.arm.ManualMoveShoulder;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.arm.ResetArmFromAbsolute;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.commands.routines.EjectAndStow;
import frc.team670.robot.commands.claw.ClawIdle;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.arm.ManualMoveElbow;
import frc.team670.robot.commands.arm.ManualMoveShoulder;
import frc.team670.robot.commands.claw.ClawEject;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.commands.drivebase.TurnToAngle;
import frc.team670.robot.commands.vision.AutoAlign;
import frc.team670.robot.commands.claw.ClawIdle;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public class OI extends OIBase {

    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);

    // Driver buttons
    // private static JoystickButton zeroArm = new JoystickButton(driverController, XboxButtons.START);
    private static JoystickButton zeroArm = new JoystickButton(operatorController, XboxButtons.START);
    private static JoystickButton zeroGyroDriver = new JoystickButton(driverController, XboxButtons.START);

    // Operator buttons
    private static POVButton hybrid = new POVButton(operatorController, 180);
    private static POVButton scoreMidR = new POVButton(operatorController, 90);
    private static POVButton singleStation = new POVButton(operatorController, 270);
    private static POVButton scoreHigh = new POVButton(operatorController, 0);

    private static JoystickButton stow = new JoystickButton(operatorController, XboxButtons.B);
    private static JoystickButton manualElbowControl = new JoystickButton(operatorController,
            XboxButtons.RIGHT_JOYSTICK_BUTTON);
    private static JoystickButton manualShoulderControl = new JoystickButton(operatorController,
            XboxButtons.LEFT_JOYSTICK_BUTTON);

    private static JoystickButton clawSuck = new JoystickButton(operatorController, XboxButtons.RIGHT_BUMPER);
    private static JoystickButton clawEject = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    private static JoystickButton clawIdle = new JoystickButton(operatorController, XboxButtons.Y);
    private static JoystickButton clawSuckDriver = new JoystickButton(driverController, XboxButtons.RIGHT_BUMPER);
    private static JoystickButton clawEjectDriver = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);


    public static MustangController getDriverController() {
        return driverController;
    }

    public static MustangController getOperatorController() {
        return operatorController;
    }

    @Override
    public void configureButtonBindings(MustangSubsystemBase... subsystemBases) {
        DriveBase driveBase = (DriveBase) subsystemBases[0];
        // Vision vision = (Vision) subsystemBases[1];
        Arm arm = (Arm) subsystemBases[2];
        Claw claw = (Claw) subsystemBases[3];

        driveBase.initDefaultCommand();

        // zeroGyro.onTrue(new SetSwerveForwardDirection(driveBase, arm));
        // moveToTarget.whileTrue(new PathFindMoveToPose(driveBase, new Pose2d((FieldConstants.Grids.complexLowTranslations[1]), new Rotation2d())));
        // moveToTarget.whileTrue(new MoveToPose(driveBase, new Pose2d((FieldConstants.Grids.complexLowTranslations[1]), new Rotation2d())));
        zeroGyroDriver.onTrue(new SetSwerveForwardDirection(driveBase));
        zeroArm.onTrue(new ResetArmFromAbsolute(arm));

        //arm movement commands
        hybrid.onTrue(new MoveToTarget(arm, ArmState.HYBRID));
        scoreMidR.onTrue(new MoveToTarget(arm, ArmState.SCORE_MID));
        singleStation.onTrue(new MoveToTarget(arm, claw, ArmState.SINGLE_STATION));
        scoreHigh.onTrue(new MoveToTarget(arm, ArmState.SCORE_HIGH));
        stow.onTrue(new MoveToTarget(arm, ArmState.STOWED));
        manualShoulderControl.onTrue(new ManualMoveShoulder(arm, operatorController));
        manualElbowControl.onTrue(new ManualMoveElbow(arm, operatorController));
        
        // Claw control commands
        clawSuck.onTrue(new ClawIntake(claw));
        clawEject.onTrue(new EjectAndStow(claw, arm));
        clawIdle.onTrue(new ClawIdle(claw));

    }

}
