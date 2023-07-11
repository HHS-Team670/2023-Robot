/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.subsystems.arm.Arm;


/**
 * RobotContainer is where we put the high-level code for the robot. It contains
 * subsystems, OI
 * devices, etc, and has required methods (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {
    // private final Vision mVision = Vision.getInstance();
    private final DriveBase mDriveBase = DriveBase.getInstance();
    private final LED mLed = LED.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Claw mClaw = Claw.getInstance();

    private MustangCommand cableScore, cableEngage, stationScore, stationEngage, centerEngage,
            centerIntake, scoreMid;

    private Notifier updateArbitraryFeedForward;

    private final String kMatchStartedString = "match-started";
    private final String kAutonChooserString = "auton-chooser";

    public RobotContainer() {
        super();
        addSubsystem(mDriveBase);
        OI.configureButtonBindings();

        for (MustangSubsystemBase subsystem : getSubsystems()) {
            subsystem.setDebugSubsystem(true);
        }

        // cableScore = new ConeCube(mDriveBase, mClaw, mArm,mLed, "CableScoreShort");
        // stationScore = new ConeCubeCube(mDriveBase, mClaw,mArm,mLed, "Station3Piece");
        // cableEngage = new CubeEngage(mDriveBase, mClaw, mArm,mLed, "CableEngage");
        // stationEngage = new ScoreEngage(mDriveBase, mClaw, mArm, mLed,"StationScoreEngage3");
        // centerEngage = new CenterEngageSequential(mDriveBase, mClaw, mArm, mLed);
        // centerIntake = new CenterIntake(mDriveBase, mClaw, mArm, mLed, "CenterIntake");
        // scoreMid = new ScoreMid(mDriveBase, mClaw, mArm, mLed);

    }

    @Override
    public void robotInit() {
        //CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kYUYV, 160, 120, 30);

        // mDriveBase.initVision(mVision);
        // SmartDashboard.putNumber(kAutonChooserString, 0);
        // updateArbitraryFeedForward = new Notifier(
        //         () -> {
        //             mArm.updateArbitraryFeedForward();
        //         });

        //updateArbitraryFeedForward.startPeriodic(0.01);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        return null;
        // SmartDashboard.putBoolean(kMatchStartedString, true);

        // int selectedPath = (int) SmartDashboard.getNumber(kAutonChooserString, 0);
        // MustangCommand autonCommand;
        // switch (selectedPath) {
        //     case 0:
        //         autonCommand = cableScore;
        //         break;
        //     case 1:
        //         autonCommand = stationScore;
        //         break;
        //     case 2:
        //         autonCommand = cableEngage;
        //         break;
        //     case 3:
        //         autonCommand = stationEngage;
        //         break;
        //     case 4:
        //         autonCommand = centerEngage;
        //         break;
        //     case 5:
        //         autonCommand = centerIntake;
        //         break;
        //     case 6:
        //         autonCommand = scoreMid;
        //         break;
        //     default:
        //         autonCommand = centerEngage;
        // }
        // mLed.updateAutonPathColor(selectedPath);
        // return autonCommand;
    }

    @Override
    public void autonomousInit() {
        mArm.setStateToStarting();
    }

    @Override
    public void teleopInit() {
        // mArm.clearSetpoint();
        // new ResetArmFromAbsolute(mArm);
    }

    @Override
    public void testInit() {
    }

    @Override
    public void disabled() {
        //SmartDashboard.putBoolean(kMatchStartedString, false);
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

    @Override
    public MustangController getOperatorController() {
        return OI.getOperatorController();
    }

    @Override
    public MustangController getDriverController() {
        return OI.getDriverController();
    }

    @Override
    public MustangController getBackupController() {
        return null;
    }

    private void parkBeforeDisable() {
        //double cTime = DriverStation.getMatchTime();
       //
    }

}
