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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.arm.ResetArmFromAbsolute;
import frc.team670.robot.commands.pathplanner.CenterEngageSequential;
import frc.team670.robot.commands.pathplanner.CenterIntake;
import frc.team670.robot.commands.pathplanner.ConeCube;
import frc.team670.robot.commands.pathplanner.ConeCubeCube;
import frc.team670.robot.commands.pathplanner.CubeEngage;
import frc.team670.robot.commands.pathplanner.ScoreEngage;
import frc.team670.robot.commands.pathplanner.ScoreMid;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.drivebase.DriveBase;

/**
 * RobotContainer is where we put the high-level code for the robot. It contains
 * subsystems, OI
 * devices, etc, and has required methods (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {
    private final Vision mVision = Vision.getInstance();
    private final DriveBase mDriveBase = DriveBase.getInstance();
    private final LED mLed = LED.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Claw mClaw = Claw.getInstance();

    private MustangCommand cableScore, cableEngage, stationScore, stationEngage, centerEngage,
            centerIntake, scoreMid, leftTwoPiece, rightTwoPiece;

    private Notifier updateArbitraryFeedForward;

    private final String kMatchStartedString = "match-started";
    private final String kAutonChooserString = "auton-chooser";

    public RobotContainer() {
        super();
        addSubsystem(mDriveBase, mVision, mArm, mArm.getShoulder(), mArm.getElbow(), mArm.getWrist(),
                mClaw, mLed);
        OI.configureButtonBindings();

        for (MustangSubsystemBase subsystem : getSubsystems()) {
            subsystem.setDebugSubsystem(true);
        }

        cableScore = new ConeCube(mDriveBase, mClaw, mArm,mLed, "CableScoreShort");
        stationScore = new ConeCubeCube(mDriveBase, mClaw,mArm,mLed, "Station3Piece");
        leftTwoPiece = new ConeCubeCube(mDriveBase, mClaw,mArm,mLed, "L2 Two-Piece Left");
        rightTwoPiece = new ConeCubeCube(mDriveBase, mClaw,mArm,mLed, "L2 Two-Piece Right");
        cableEngage = new CubeEngage(mDriveBase, mClaw, mArm,mLed, "CableEngage");
        stationEngage = new ScoreEngage(mDriveBase, mClaw, mArm, mLed,"test");
        centerEngage = new CenterEngageSequential(mDriveBase, mClaw, mArm, mLed);
        centerIntake = new CenterIntake(mDriveBase, mClaw, mArm, mLed, "CenterIntake");
        scoreMid = new ScoreMid(mDriveBase, mClaw, mArm, mLed);

    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kYUYV, 160, 120, 30);

        mDriveBase.initVision(mVision);
        SmartDashboard.putNumber(kAutonChooserString, 0);
        updateArbitraryFeedForward = new Notifier(
                () -> {
                    mArm.updateArbitraryFeedForward();
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
        SmartDashboard.putBoolean(kMatchStartedString, true);

        int selectedPath = (int) SmartDashboard.getNumber(kAutonChooserString, 0);
        MustangCommand autonCommand;
        switch (selectedPath) {
            case 0:
                autonCommand = cableScore;
                break;
            case 1:
                autonCommand = stationScore;
                break;
            case 2:
                autonCommand = cableEngage;
                break;
            case 3:
                autonCommand = stationEngage;
                break;
            case 4:
                autonCommand = centerEngage;
                break;
            case 5:
                autonCommand = centerIntake;
                break;
            case 6:
                autonCommand = scoreMid;
                break;
            default:
                autonCommand = centerEngage;
        }
        mLed.updateAutonPathColor(selectedPath);
        return stationEngage;
    }

    @Override
    public void autonomousInit() {
        mArm.setStateToStarting();
    }

    @Override
    public void teleopInit() {
        mArm.clearSetpoint();
        new ResetArmFromAbsolute(mArm);
    }

    @Override
    public void testInit() {
    }

    @Override
    public void disabled() {
        SmartDashboard.putBoolean(kMatchStartedString, false);
    }

    @Override
    public void disabledPeriodic() {
        mArm.getShoulder().sendAngleToDashboard();
        mArm.getElbow().sendAngleToDashboard();
        mArm.getWrist().sendAngleToDashboard();

        int selectedPath = (int) SmartDashboard.getNumber(kAutonChooserString, 0);
        mLed.updateAutonPathColor(selectedPath);
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
