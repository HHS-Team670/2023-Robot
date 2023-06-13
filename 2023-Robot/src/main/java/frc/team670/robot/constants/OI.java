package frc.team670.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.commands.drive.teleop.XboxSwerveDrive;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.arm.ManualMoveElbow;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.arm.ResetArmFromAbsolute;
import frc.team670.robot.commands.claw.ClawIdle;
import frc.team670.robot.commands.claw.ClawInstantIntake;
import frc.team670.robot.commands.leds.SetColorPurple;
import frc.team670.robot.commands.leds.SetColorYellow;
import frc.team670.robot.commands.routines.EjectAndStow;
import frc.team670.robot.commands.vision.AutoAlign;
import frc.team670.robot.commands.vision.AutoAlignToSubstation;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

public final class OI {

    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);

    // Driver buttons
    private static JoystickButton zeroArm =
            new JoystickButton(operatorController, XboxButtons.START);
    private static JoystickButton zeroGyroDriver =
            new JoystickButton(driverController, XboxButtons.START);
    private static POVButton singleSubAlign = new POVButton(driverController, 0);
    private static POVButton alignToClosest = new POVButton(driverController, 180);
    private static POVButton alignToLeft = new POVButton(driverController, 270);
    private static POVButton alignToRight = new POVButton(driverController, 90);

    // Operator buttons
    private static POVButton hybrid = new POVButton(operatorController, 180);
    private static POVButton scoreMidR = new POVButton(operatorController, 90);
    private static POVButton singleStation = new POVButton(operatorController, 270);
    private static POVButton scoreHigh = new POVButton(operatorController, 0);
    private static JoystickButton intakeShelf =
            new JoystickButton(operatorController, XboxButtons.X);
    private static JoystickButton uprightGround =
            new JoystickButton(operatorController, XboxButtons.BACK);

    private static JoystickButton stow = new JoystickButton(operatorController, XboxButtons.B);
    private static JoystickButton manualElbowControlPositive =
            new JoystickButton(operatorController, XboxButtons.RIGHT_JOYSTICK_BUTTON);
    private static JoystickButton manualElbowControlNegative =
            new JoystickButton(operatorController, XboxButtons.LEFT_JOYSTICK_BUTTON);

    private static JoystickButton clawSuck =
            new JoystickButton(operatorController, XboxButtons.RIGHT_BUMPER);
    private static JoystickButton clawEject =
            new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    private static JoystickButton clawIdle =
            new JoystickButton(operatorController, XboxButtons.LEFT_BUMPER);

    // Align to cardinal directions
    private static JoystickButton rotateTo0 = new JoystickButton(driverController, XboxButtons.Y);
    private static JoystickButton rotateTo90 = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton rotateTo180 = new JoystickButton(driverController, XboxButtons.A);
    private static JoystickButton rotateTo270 = new JoystickButton(driverController, XboxButtons.B);

    // LED commands
    private static JoystickButton ledYellow = new JoystickButton(operatorController, XboxButtons.Y);
    private static JoystickButton ledPurple = new JoystickButton(operatorController, XboxButtons.A);

    public static MustangController getDriverController() {
        return driverController;
    }

    public static MustangController getOperatorController() {
        return operatorController;
    }

    public static void configureButtonBindings() {
        DriveBase driveBase = DriveBase.getInstance();
        Arm arm = Arm.getInstance();
        Claw claw = Claw.getInstance();
        LED led = LED.getInstance();

        driveBase.initDefaultCommand(new XboxSwerveDrive(driveBase, driverController,
                RobotConstants.DriveBase.kMaxVelocityMetersPerSecond,
                RobotConstants.DriveBase.kMaxAngularVelocityRadiansPerSecond));

        zeroGyroDriver.onTrue(new SetSwerveForwardDirection(driveBase));
        zeroArm.onTrue(new ResetArmFromAbsolute(arm));
        singleSubAlign.whileTrue(new AutoAlignToSubstation(driveBase, false)); // moves to
                                                                               // substation

        alignToClosest.whileTrue(new AutoAlign(driveBase, AutoAlign.Direction.CLOSEST));
        alignToLeft.whileTrue(new AutoAlign(driveBase, AutoAlign.Direction.LEFT));
        alignToRight.whileTrue(new AutoAlign(driveBase, AutoAlign.Direction.RIGHT));

        // arm movement commands
        hybrid.onTrue(new MoveToTarget(arm, ArmState.HYBRID));
        scoreMidR.onTrue(new MoveToTarget(arm, ArmState.SCORE_MID));
        singleStation.onTrue(new MoveToTarget(arm, claw, ArmState.SINGLE_STATION));
        scoreHigh.onTrue(new MoveToTarget(arm, ArmState.SCORE_HIGH));
        intakeShelf.onTrue(new MoveToTarget(arm, claw, ArmState.INTAKE_SHELF));
        uprightGround.onTrue(new MoveToTarget(arm, ArmState.UPRIGHT_GROUND));
        stow.onTrue(new MoveToTarget(arm, ArmState.STOWED));
        manualElbowControlNegative.onTrue(new ManualMoveElbow(arm, false));
        manualElbowControlPositive.onTrue(new ManualMoveElbow(arm, true));

        // Claw control commands
        clawSuck.onTrue(new ClawInstantIntake(claw));
        clawEject.onTrue(new EjectAndStow(claw, arm));
        clawIdle.onTrue(new ClawIdle(claw));

        // Rotate to cardinal direction while driving
        XboxSwerveDrive driveCommand = (XboxSwerveDrive) driveBase.getDefaultCommand();
        rotateTo0.onTrue(driveCommand.new SetDesiredHeading(new Rotation2d(0)));
        rotateTo90.onTrue(driveCommand.new SetDesiredHeading(new Rotation2d(Math.PI / 2)));
        rotateTo180.onTrue(driveCommand.new SetDesiredHeading(new Rotation2d(Math.PI)));
        rotateTo270.onTrue(driveCommand.new SetDesiredHeading(new Rotation2d(3 * Math.PI / 2)));

        ledPurple.onTrue(new SetColorPurple(led));
        ledYellow.onTrue(new SetColorYellow(led));
    }
}
