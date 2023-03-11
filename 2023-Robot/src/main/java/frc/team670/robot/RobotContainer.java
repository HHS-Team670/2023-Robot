/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.drivebase.NonPidAutoLevel;
import frc.team670.robot.commands.drivebase.SwerveDriveParkCommand;
import frc.team670.robot.commands.pathplanner.AutonCalibration;
import frc.team670.robot.commands.pathplanner.CenterEngage;
import frc.team670.robot.commands.pathplanner.CenterIntake;
import frc.team670.robot.commands.pathplanner.ConeCube;
import frc.team670.robot.commands.pathplanner.CubeEngage;
import frc.team670.robot.commands.pathplanner.ScoreMid;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotMap;
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

    private final Vision vision = new Vision(pd);
    private final DriveBase driveBase = new DriveBase(getDriverController());
    private final Arm arm = new Arm();
    private final LED led = new LED(RobotMap.LED_PORT, 0, 35);
    private final Claw claw = new Claw(led);

    private static OI oi = new OI();
    private Notifier updateArbitraryFeedForward;


    public RobotContainer() {
        super();
        addSubsystem(driveBase, vision, arm, arm.getShoulder(), arm.getElbow(), arm.getWrist(),
                claw, led);
        oi.configureButtonBindings(driveBase, vision, arm, claw, led);

        arm.getElbow().setDebugSubsystem(true);
        // for (MustangSubsystemBase subsystem : getSubsystems()) {
        //     subsystem.setDebugSubsystem(true);
        // }

    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kYUYV, 160, 120, 30);

        driveBase.initVision(vision);

        updateArbitraryFeedForward = new Notifier(new Runnable() {
            public void run() {
                arm.updateArbitraryFeedForward();
            }
        });
        led.rainbow(false);

        updateArbitraryFeedForward.startPeriodic(0.01);

        Alliance alliance = DriverStation.getAlliance();

        if (alliance == Alliance.Red) {
            SmartDashboard.putString("Alliance Color", "red");
            led.setAllianceColors(LEDColor.RED);
        } else if (alliance == Alliance.Blue) {
            SmartDashboard.putString("Alliance Color", "blue");
            led.setAllianceColors(LEDColor.BLUE);
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        // return new NonPidAutoLevel(driveBase, true);
        
        SmartDashboard.putBoolean("match-started", true);

        int selectedPath = (int) (SmartDashboard.getEntry("auton-chooser").getInteger(-1));
        MustangCommand autonCommand;
        switch (selectedPath) {
            case 0:
                autonCommand = new ConeCube(driveBase, claw, arm, "CableScoreShort");
                led.solidhsv(led.getAllianceColor());
                break;
            case 1:
                autonCommand = new ConeCube(driveBase, claw, arm, "StationScoreShort");
                led.solidrgb(LEDColor.SEXY_YELLOW);
                break;
            case 2:
                autonCommand = new CubeEngage(driveBase, claw, arm, "CableEngage");
                led.solidrgb(LEDColor.SEXY_PURPLE);
                break;
            case 3:
                autonCommand = new CubeEngage(driveBase, claw, arm, "StationEngage");
                led.solidhsv(LEDColor.LIGHT_BLUE); 
                break;
            case 4:
                autonCommand = new CenterEngage(driveBase, claw, arm, "CenterEngage");
                led.rainbow(false);
                break;
            case 5:
                autonCommand = new CenterIntake(driveBase, claw, arm, "CenterIntake");
                led.solidrgb(LEDColor.WHITE);
                break;
            case 6:
                autonCommand = new ScoreMid(driveBase, claw, arm);
                led.mustangRainbow();
                break;
            default:
                autonCommand = new CenterEngage(driveBase, claw, arm, "CenterEngage");
                led.rainbow(false);

        }
        return autonCommand;

        // LEAVE COMMENTED
        //greturn new ConeCube(driveBase, claw, arm, "CableScore");
        // return new AutonCalibration(driveBase, "Curve"); // TODO: use curve path after straight
        // path

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
        // arm.setStateToStarting();
        led.solidhsv(led.getAllianceColor());
        arm.clearSetpoint();
    }

    @Override
    public void testInit() {}

    @Override
    public void disabled() {
        SmartDashboard.putBoolean("match-started", false);
        led.rainbow(false);
    }


    @Override
    public void disabledPeriodic() {
        int selectedPath = (int) (SmartDashboard.getEntry("auton-chooser").getInteger(-1));
        switch (selectedPath) {
            case 0:
                led.solidhsv(led.getAllianceColor());
                break;
            case 1:
                led.solidrgb(LEDColor.SEXY_YELLOW);
                break;
            case 2:
                led.solidrgb(LEDColor.SEXY_PURPLE);
                break;
            case 3:
                led.solidhsv(LEDColor.LIGHT_BLUE); 
                break;
            case 4:
                led.rainbow(false);
                break;
            case 5:
                led.solidrgb(LEDColor.WHITE);
                break;
            case 6: 
                led.mustangRainbow();
                break;
            default:
                led.rainbow(false);

        }
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        // SmartDashboard.putString("alliance", "" +
        // DriverStationJNI.getAllianceStation());
    }

    @Override
    public void autonomousPeriodic() {

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

}
