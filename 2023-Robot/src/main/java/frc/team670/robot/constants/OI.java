package frc.team670.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.team670.mustanglib.commands.drive.teleop.SetSwerveForwardDirection;
import frc.team670.mustanglib.constants.OIBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.MustangController.XboxButtons;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.commands.claw.ClawEject;
import frc.team670.robot.commands.claw.ClawIntake;
import frc.team670.robot.commands.drivebase.MoveToPose;
import frc.team670.robot.commands.drivebase.PathFindMoveToPose;
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
    private static JoystickButton moveToTarget = new JoystickButton(driverController, XboxButtons.Y);

    //private static JoystickButton move = new JoystickButton(driverController, XboxButtons.Y);

    // Operator buttons
    private static POVButton backward = new POVButton(operatorController, 180);
    private static POVButton scoreMid = new POVButton(operatorController, 90);
    private static POVButton scoreHigh = new POVButton(operatorController, 0);
    private static JoystickButton clawSuck = new JoystickButton(operatorController, XboxButtons.A);
    private static JoystickButton clawEject = new JoystickButton(operatorController, XboxButtons.B);



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
        // vision.initDefaultCommand(new IsLockedOn(driveBase, vision, null));

        zeroGyro.onTrue(new SetSwerveForwardDirection(driveBase, arm));
        // moveToTarget.whileTrue(new PathFindMoveToPose(driveBase, new Pose2d(FieldConstants.allianceFlip(FieldConstants.Grids.complexLowTranslations[1]), new Rotation2d())));
        moveToTarget.whileTrue(new MoveToPose(driveBase, new Pose2d(FieldConstants.allianceFlip(FieldConstants.Grids.complexLowTranslations[1]), new Rotation2d())));

        // Arm movement commands
        backward.onTrue(new MoveToTarget(arm, ArmState.BACKWARD_GROUND));
        scoreMid.onTrue(new MoveToTarget(arm, ArmState.SCORE_MID));
        scoreHigh.onTrue(new MoveToTarget(arm, ArmState.SCORE_HIGH));

        backward.onFalse(new MoveToTarget(arm, ArmState.STOWED));
        scoreMid.onFalse(new MoveToTarget(arm, ArmState.STOWED));
        scoreHigh.onFalse(new MoveToTarget(arm, ArmState.STOWED));

        //Claw control commands
        clawSuck.onTrue(new ClawIntake(claw));
        clawEject.onTrue(new ClawEject(claw));
        
    }

}
