/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import java.lang.reflect.Field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
import frc.team670.robot.commands.drivebase.SwerveDriveParkCommand;
import frc.team670.robot.commands.pathplanner.AutonCalibration;
import frc.team670.robot.commands.pathplanner.CenterEngage;
import frc.team670.robot.commands.pathplanner.CenterIntake;
import frc.team670.robot.commands.pathplanner.ConeCubeCube;
import frc.team670.robot.commands.pathplanner.ConeCubeEngage;
import frc.team670.robot.commands.pathplanner.CubeEngage;
import frc.team670.robot.commands.pathplanner.ScoreEngage;
import frc.team670.robot.commands.pathplanner.ScoreMid;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.subsystems.arm.Arm;

/**
 * RobotContainer is where we put the high-level code for the robot. It contains
 * subsystems, OI
 * devices, etc, and has required methods (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {

    private final PowerDistribution pd = new PowerDistribution(1, ModuleType.kCTRE);

    private final Vision vision = new Vision(pd);
    private final DriveBase driveBase = new DriveBase(getDriverController());
    private final Arm arm = new Arm();
    private final LED led = new LED(RobotMap.LED_PORT, 0, 61);
    private final Claw claw = new Claw(led);

    private MustangCommand cableScore, cableEngage, stationScore, stationEngage, centerEngage,
            centerIntake, scoreMid;

    private static OI oi = new OI();
    private Notifier updateArbitraryFeedForward;

    private final String matchStarted = "match-started";
    private final String autonChooser = "auton-chooser";

    public RobotContainer() {
        super();
        addSubsystem(driveBase, vision, arm, arm.getShoulder(), arm.getElbow(), arm.getWrist(),
                claw, led);
        oi.configureButtonBindings(driveBase, vision, arm, claw, led);

        // for (MustangSubsystemBase subsystem : getSubsystems()) {
        // subsystem.setDebugSubsystem(true);
        // }
        arm.getWrist().setDebugSubsystem(true);

        cableScore = new ConeCubeCube(driveBase, claw, arm, "CableScoreShort");
        stationScore = new ConeCubeCube(driveBase, claw, arm, "Station3Piece");
        cableEngage = new CubeEngage(driveBase, claw, arm, "CableEngage");
        stationEngage = new ScoreEngage(driveBase, claw, arm, "StationScoreEngage3");
        centerEngage = new CenterEngage(driveBase, claw, arm, "CenterEngage");
        centerIntake = new CenterIntake(driveBase, claw, arm, "CenterIntake");
        scoreMid = new ScoreMid(driveBase, claw, arm);

    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kYUYV, 160, 120, 30);

        driveBase.initVision(vision);
        SmartDashboard.putNumber(autonChooser, 0);
        updateArbitraryFeedForward = new Notifier(new Runnable() {
            public void run() {
                arm.updateArbitraryFeedForward();
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

        SmartDashboard.putBoolean(matchStarted, true);

        // SmartDashboard.putNumber(autonChooser, 0);
        int selectedPath = (int) SmartDashboard.getNumber(autonChooser, 0);
        MustangCommand autonCommand;
        switch (selectedPath) {
            case 0:
                autonCommand = cableScore;
                led.solidhsv(LEDColor.LIGHT_BLUE);
                break;
            case 1:
                autonCommand = stationScore;
                led.solidhsv(LEDColor.SEXY_YELLOW);
                break;
            case 2:
                autonCommand = cableEngage;
                led.solidhsv(LEDColor.SEXY_PURPLE);
                break;
            case 3:
                autonCommand = stationEngage;
                led.solidhsv(LEDColor.GREEN);
                break;
            case 4:
                autonCommand = centerEngage;
                led.animatedRainbow(false, 10, 10);
                break;
            case 5:
                autonCommand = centerIntake;
                led.solidhsv(LEDColor.PINK);
                break;
            case 6:
                autonCommand = scoreMid;
                led.animatedMustangRainbow(10, 10);
                break;
            default:
                autonCommand = centerEngage;
                led.animatedRainbow(false, 10, 10);

        }
        return autonCommand;

        // LEAVE COMMENTED
        // greturn new ConeCube(driveBase, claw, arm, "CableScore");
        //return new AutonCalibration(driveBase, "Straight180"); // TODO: use curve
        // path after
        // straight
        // path
        // return new ConeCubeCube(driveBase, claw, arm, "Station3Piece");
        // return new ConeCubeEngage(driveBase, claw, arm, "StationScoreEngage2");
        // return new NonPidAutoLevel(driveBase, true);
        //return new CenterEngage(driveBase, claw, arm, "CenterEngage");

        // return new ConeCube(driveBase, claw, arm, "CableScore");
        // return new ConeCube(driveBase, claw, arm, "RightConeCube");
        // return new NonPidAutoLevel(driveBase, true);
    }

    @Override
    public void autonomousInit() {
        arm.setStateToStarting();
    }

    @Override
    public void teleopInit() {
        vision.setAprilTagFieldLayout(FieldConstants.getFieldLayout(FieldConstants.aprilTags));
        arm.clearSetpoint();
    }

    @Override
    public void testInit() {
    }

    @Override
    public void disabled() {
        SmartDashboard.putBoolean(matchStarted, false);
    }

    @Override
    public void disabledPeriodic() {
        // int selectedPath = (int)
        // (SmartDashboard.getEntry(autonChooser).getInteger(-1));

        int selectedPath = (int) SmartDashboard.getNumber(autonChooser, 0);
        switch (selectedPath) {
            case 0:
                led.blinkhsv(LEDColor.LIGHT_BLUE);
                break;
            case 1:
                led.blinkhsv(LEDColor.SEXY_YELLOW);
                break;
            case 2:
                led.blinkhsv(LEDColor.SEXY_PURPLE);
                break;
            case 3:
                led.blinkhsv(LEDColor.GREEN);
                break;
            case 4:
                led.animatedRainbow(false, 10, 10);
                break;
            case 5:
                led.blinkhsv(LEDColor.PINK);
                break;
            case 6:
                led.animatedMustangRainbow(10, 10);
                break;
            default:
                led.animatedRainbow(false, 10, 10);

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
            driveBase.park();
        }
    }

}
