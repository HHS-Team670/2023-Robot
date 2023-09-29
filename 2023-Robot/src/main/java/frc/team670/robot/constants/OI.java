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
import frc.team670.robot.commands.leds.SetIntakeCube;
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

    // Driver buttons
    private static JoystickButton resetManualOffset =
            new JoystickButton(operatorController, XboxButtons.START);
    private static JoystickButton zeroGyroDriver =
            new JoystickButton(driverController, XboxButtons.START);
    private static POVButton singleSubAlign = new POVButton(driverController, 0);
    private static POVButton alignToClosest = new POVButton(driverController, 180);
    private static POVButton alignToLeft = new POVButton(driverController, 270);
    private static POVButton alignToRight = new POVButton(driverController, 90);

    // Operator buttons
    private static POVButton uprightGround = new POVButton(operatorController, 180);
    private static POVButton scoreMidR = new POVButton(operatorController, 90);
    private static POVButton intakeShelf = new POVButton(operatorController, 270);
    private static POVButton scoreHigh = new POVButton(operatorController, 0);
    private static JoystickButton zeroArm =
            new JoystickButton(operatorController, XboxButtons.X);
    private static JoystickButton clawIdle =
            new JoystickButton(operatorController, XboxButtons.BACK);

    private static JoystickButton stow = new JoystickButton(operatorController, XboxButtons.B);
    private static JoystickButton manualElbowControlPositive =
            new JoystickButton(operatorController, XboxButtons.RIGHT_JOYSTICK_BUTTON);
    private static JoystickButton manualElbowControlNegative =
            new JoystickButton(operatorController, XboxButtons.LEFT_JOYSTICK_BUTTON);

    private static JoystickButton suck =
            new JoystickButton(operatorController, XboxButtons.RIGHT_BUMPER);
    private static JoystickButton eject =
            new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    private static JoystickButton toggleCubeIntake= new JoystickButton(operatorController, XboxButtons.LEFT_BUMPER);
    private static JoystickButton turnToCone= new JoystickButton(driverController,XboxButtons.RIGHT_BUMPER);


    // Align to cardinal directions
    private static JoystickButton rotateTo0 = new JoystickButton(driverController, XboxButtons.Y);
    private static JoystickButton rotateTo90 = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton rotateTo180 = new JoystickButton(driverController, XboxButtons.A);
    private static JoystickButton rotateTo270 = new JoystickButton(driverController, XboxButtons.B);

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
        DriveBase driveBase = DriveBase.getInstance();
        Arm arm = Arm.getInstance();
        Claw claw = Claw.getInstance();
        LED led = LED.getInstance();
        CubeIntake cubeIntake = CubeIntake.getInstance();

        driveBase.initDefaultCommand(new XboxSwerveDrive(driveBase, driverController));

        zeroGyroDriver.onTrue(new SetSwerveForwardDirection(driveBase));
        zeroArm.onTrue(new ResetArmFromAbsolute(arm));
        singleSubAlign.whileTrue(new AutoAlignToSubstation(driveBase, false)); // moves to
        resetManualOffset.onTrue(new ResetArmOffset(arm));                                   // substation

        alignToClosest.whileTrue(new AutoAlign(driveBase, AutoAlign.Direction.CLOSEST));
        alignToLeft.whileTrue(new AutoAlign(driveBase, AutoAlign.Direction.LEFT));
        alignToRight.whileTrue(new AutoAlign(driveBase, AutoAlign.Direction.RIGHT));

        // arm movement commands
  
        scoreMidR.onTrue(new MoveToTarget(arm, ArmState.SCORE_MID));
  
        scoreHigh.onTrue(new MoveToTarget(arm, ArmState.SCORE_HIGH));
        intakeShelf.onTrue(new MoveToTarget(arm, claw, ArmState.INTAKE_SHELF));
        uprightGround.onTrue(new MoveToTarget(arm, ArmState.UPRIGHT_GROUND));
        stow.onTrue(new MoveToTarget(arm, ArmState.STOWED));
        manualElbowControlNegative.onTrue(new ManualMoveWrist(arm, false));
        manualElbowControlPositive.onTrue(new ManualMoveWrist(arm, true));

        // Claw control commands
        suck.onTrue(new DualIntake(claw,arm,cubeIntake));
        eject.onTrue(new DualEject(claw, arm, cubeIntake));
        clawIdle.onTrue(new DualIdle(claw, cubeIntake));
        toggleCubeIntake.onTrue(new ToggleCubeIntakeDeployer(cubeIntake));
        // Rotate to cardinal direction while driving
        XboxSwerveDrive driveCommand = (XboxSwerveDrive) driveBase.getDefaultCommand();
        rotateTo0.onTrue(driveCommand.new SetDesiredHeading(new Rotation2d(0)));
        rotateTo90.onTrue(driveCommand.new SetDesiredHeading(new Rotation2d(Math.PI / 2)));
        rotateTo180.onTrue(driveCommand.new SetDesiredHeading(new Rotation2d(Math.PI)));
        rotateTo270.onTrue(driveCommand.new SetDesiredHeading(new Rotation2d(3 * Math.PI / 2)));

        cubeSuck.onTrue(new SetIntakeCube(led, claw));
        coneSuck.onTrue(new SetIntakeCone(led, claw));
        turnToCone.onTrue(new MoveToCone(driveBase, driverController));
    }
}
