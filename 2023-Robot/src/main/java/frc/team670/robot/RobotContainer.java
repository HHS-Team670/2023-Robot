/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.pathplanner.CenterEngageSequential;
import frc.team670.robot.commands.pathplanner.CenterIntake;
import frc.team670.robot.commands.pathplanner.ConeCube;
import frc.team670.robot.commands.pathplanner.ConeCubeCube;
import frc.team670.robot.commands.pathplanner.CubeEngage;
import frc.team670.robot.commands.pathplanner.ScoreEngage;
import frc.team670.robot.commands.pathplanner.ScoreMid;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.subsystems.arm.Arm;

/**
 * RobotContainer is where we put the high-level code for the robot. It contains subsystems, OI
 * devices, etc, and has required methods (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {

    private final PowerDistribution pd = new PowerDistribution(1, ModuleType.kCTRE);

    private final Vision mVision = new Vision(pd);
    private final DriveBase mDriveBase = new DriveBase();
    private final LED mLed = new LED(RobotConstants.kLEDPort, 0, 61);
    private final Arm mArm = new Arm();
    private final Claw mClaw = new Claw(mLed);

    private MustangCommand cableScore, cableEngage, stationScore, stationEngage, centerEngage,
            centerIntake, scoreMid;

    private static OI kOi = new OI();
    private Notifier updateArbitraryFeedForward;

    private final String kMatchStartedString = "match-started";
    private final String kAutonChooserString = "auton-chooser";

    public RobotContainer() {
        super();
        addSubsystem(mDriveBase, mVision, mArm, mArm.getShoulder(), mArm.getElbow(), mArm.getWrist(),
                mClaw, mLed);
        kOi.configureButtonBindings(mDriveBase, mVision, mArm, mClaw, mLed);

        for (MustangSubsystemBase subsystem : getSubsystems()) {
            subsystem.setDebugSubsystem(true);
        }

        cableScore = new ConeCube(mDriveBase, mClaw, mArm, "CableScoreShort");
        stationScore = new ConeCubeCube(mDriveBase, mClaw, mArm, "Station3Piece");
        cableEngage = new CubeEngage(mDriveBase, mClaw, mArm, "CableEngage");
        stationEngage = new ScoreEngage(mDriveBase, mClaw, mArm, "StationScoreEngage3");
        centerEngage = new CenterEngageSequential(mDriveBase, mClaw, mArm);
        centerIntake = new CenterIntake(mDriveBase, mClaw, mArm, "CenterIntake");
        scoreMid = new ScoreMid(mDriveBase, mClaw, mArm);

    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kYUYV, 160, 120, 30);

        mDriveBase.initVision(mVision);
        SmartDashboard.putNumber(kAutonChooserString, 0);
        updateArbitraryFeedForward = new Notifier(new Runnable() {
            public void run() {
                mArm.updateArbitraryFeedForward();
            }
        });

        updateArbitraryFeedForward.startPeriodic(0.01);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        // return new NonPidAutoLevel(driveBase, true);
        // return new ConeCube(driveBase, claw, arm, "StationScoreShort");

        SmartDashboard.putBoolean(kMatchStartedString, true);

        // SmartDashboard.putNumber(autonChooser, 0);
        int selectedPath = (int) SmartDashboard.getNumber(kAutonChooserString, 0);
        MustangCommand autonCommand;
        switch (selectedPath) {
            case 0:
                autonCommand = cableScore;
                mLed.solidhsv(LEDColor.LIGHT_BLUE);
                break;
            case 1:
                autonCommand = stationScore;
                mLed.solidhsv(LEDColor.SEXY_YELLOW);
                break;
            case 2:
                autonCommand = cableEngage;
                mLed.solidhsv(LEDColor.SEXY_PURPLE);
                break;
            case 3:
                autonCommand = stationEngage;
                mLed.solidhsv(LEDColor.GREEN);
                break;
            case 4:
                autonCommand = centerEngage;
                mLed.animatedRainbow(false, 10, 10);
                break;
            case 5:
                autonCommand = centerIntake;
                mLed.solidhsv(LEDColor.PINK);
                break;
            case 6:
                autonCommand = scoreMid;
                mLed.animatedMustangRainbow(10, 10);
                break;
            default:
                autonCommand = centerEngage;
                mLed.animatedRainbow(false, 10, 10);

        }
        return autonCommand;

        // LEAVE COMMENTED
        // greturn new ConeCube(driveBase, claw, arm, "CableScore");
        // return new AutonCalibration(driveBase, "Straight180"); // TODO: use curve
        // path after
        // straight
        // path
        // return new ConeCubeCube(driveBase, claw, arm, "Station3Piece");
        // return new ConeCubeEngage(driveBase, claw, arm, "StationScoreEngage2");
        // return new NonPidAutoLevel(driveBase, true);
        // return new CenterEngage(driveBase, claw, arm, "CenterEngage");

        // return new ConeCube(driveBase, claw, arm, "CableScore");
        // return new ConeCube(driveBase, claw, arm, "RightConeCube");
        // return new NonPidAutoLevel(driveBase, true);
    }

    @Override
    public void autonomousInit() {
        mArm.setStateToStarting();
    }

    @Override
    public void teleopInit() {
        mArm.clearSetpoint();
    }

    @Override
    public void testInit() {}

    @Override
    public void disabled() {
        SmartDashboard.putBoolean(kMatchStartedString, false);
    }

    @Override
    public void disabledPeriodic() {
        // int selectedPath = (int)
        // (SmartDashboard.getEntry(autonChooser).getInteger(-1));

        mArm.getShoulder().sendAngleToDashboard();
        mArm.getElbow().sendAngleToDashboard();
        mArm.getWrist().sendAngleToDashboard();

        int selectedPath = (int) SmartDashboard.getNumber(kAutonChooserString, 0);
        switch (selectedPath) {
            case 0:
                mLed.blinkhsv(LEDColor.LIGHT_BLUE);
                break;
            case 1:
                mLed.blinkhsv(LEDColor.SEXY_YELLOW);
                break;
            case 2:
                mLed.blinkhsv(LEDColor.SEXY_PURPLE);
                break;
            case 3:
                mLed.blinkhsv(LEDColor.GREEN);
                break;
            case 4:
                mLed.animatedRainbow(false, 10, 10);
                break;
            case 5:
                mLed.blinkhsv(LEDColor.PINK);
                break;
            case 6:
                mLed.animatedMustangRainbow(10, 10);
                break;
            default:
                mLed.animatedRainbow(false, 10, 10);

        }
    }

    @Override
    public void periodic() {
        // double cTime = DriverStation.getMatchTime();
        // if (cTime <= 0.1 && cTime != -1) {
        // driveBase.park();
        // }

        // SmartDashboard.putString("alliance", "" +

    }

    @Override
    public void autonomousPeriodic() {
        parkBeforeDisable();
    }

    @Override
    public void teleopPeriodic() {
        parkBeforeDisable();
    }


    public MustangController getOperatorController() {
        return OI.getOperatorController();
    }

    public MustangController getDriverController() {
        return OI.getDriverController();
    }

    public MustangController getBackupController() {
        return null;
    }

    private void parkBeforeDisable() {
        double cTime = DriverStation.getMatchTime();
        if (cTime <= 0.1 && cTime != -1) {
            mDriveBase.park();
        }
    }

}
