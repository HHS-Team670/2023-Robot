package frc.team670.robot.constants;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import frc.team670.robot.commands.drivebase.TurnToAngle;
import frc.team670.robot.commands.leds.SetColorPurple;
import frc.team670.robot.commands.leds.SetColorYellow;
import frc.team670.robot.commands.routines.EjectAndStow;
import frc.team670.robot.commands.vision.AutoAlign;
import frc.team670.robot.commands.claw.ClawIdle;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.commands.drivebase.Creep;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.commands.drivebase.SetDesiredHeading;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;
import frc.team670.robot.subsystems.LED;

public class OI extends OIBase {

    // Controllers
    private static MustangController driverController = new MustangController(0);
    private static MustangController operatorController = new MustangController(1);

    // Driver buttons
    // private static JoystickButton zeroArm = new JoystickButton(driverController, XboxButtons.START);
    private static JoystickButton zeroArm = new JoystickButton(operatorController, XboxButtons.START);
    private static JoystickButton zeroGyroDriver = new JoystickButton(driverController, XboxButtons.START);
    private static JoystickButton moveToTarget = new JoystickButton(driverController, XboxButtons.RIGHT_BUMPER);
    // private static JoystickButton creep = new JoystickButton(driverController, XboxButtons.RIGHT_TRIGGER);
    // private static POVButton creep = new POVButton(driverController, 0);
    private static POVButton alignToClosest = new POVButton(driverController, 0);
    private static POVButton alignToLeft = new POVButton(driverController, 90);
    private static POVButton alignToRight = new POVButton(driverController, 180);

    // private static POVButton alignToClosest = new POVButton(driverController, 0);
    
    // private static JoystickButton singleSubstation = new JoystickButton(driverController, 0)

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
    private static JoystickButton clawIdle = new JoystickButton(operatorController, XboxButtons.LEFT_BUMPER);

    //Align to cardinal directions
    private static JoystickButton rotateTo0 = new JoystickButton(driverController, XboxButtons.Y);
    private static JoystickButton rotateTo90 = new JoystickButton(driverController, XboxButtons.X);
    private static JoystickButton rotateTo180 = new JoystickButton(driverController, XboxButtons.A);
    private static JoystickButton rotateTo270 = new JoystickButton(driverController, XboxButtons.B);

    //LED commands
    private static JoystickButton ledYellow = new JoystickButton(operatorController, XboxButtons.Y);
    private static JoystickButton ledPurple = new JoystickButton(operatorController, XboxButtons.A);

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
        LED led = (LED) subsystemBases[4];
        driveBase.initDefaultCommand();

        zeroGyroDriver.onTrue(new SetSwerveForwardDirection(driveBase));
        zeroArm.onTrue(new ResetArmFromAbsolute(arm));
        moveToTarget.whileTrue(new MoveToPose(driveBase, (FieldConstants.LoadingZone.IntakePoses[0]))); // moves to substation
        
        alignToClosest.onTrue(new AutoAlign(driveBase, driverController, AutoAlign.DIRECTION.CLOSEST));
        alignToLeft.onTrue(new AutoAlign(driveBase, driverController, AutoAlign.DIRECTION.LEFT));
        alignToRight.onTrue(new AutoAlign(driveBase, driverController, AutoAlign.DIRECTION.RIGHT));

        
        // creep.whileTrue(new Creep(driveBase));
        

        

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


        //Rotate to cardinal direction while driving
        rotateTo0.onTrue(new SetDesiredHeading(driveBase, new Rotation2d(0)));
        rotateTo90.onTrue(new SetDesiredHeading(driveBase, new Rotation2d(Math.PI/2)));
        rotateTo180.onTrue(new SetDesiredHeading(driveBase, new Rotation2d(Math.PI)));
        rotateTo270.onTrue(new SetDesiredHeading(driveBase, new Rotation2d(3*Math.PI/2)));
        // rotateTo0.onTrue(new TurnToAngle(driveBase, 0, false, driverController));
        // rotateTo90.onTrue(new TurnToAngle(driveBase, 90, false, driverController));
        // rotateTo180.onTrue(new TurnToAngle(driveBase, 180, false, driverController));
        // rotateTo270.onTrue(new TurnToAngle(driveBase, 270, false, driverController));
        ledPurple.onTrue(new SetColorPurple(led));
        ledYellow.onTrue(new SetColorYellow(led));

    }
}
