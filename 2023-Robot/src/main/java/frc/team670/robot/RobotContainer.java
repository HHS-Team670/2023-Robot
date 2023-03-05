/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.drivebase.SwerveDriveParkCommand;
import frc.team670.robot.commands.pathplanner.AutonCalibration;
import frc.team670.robot.commands.pathplanner.CenterEngage;
import frc.team670.robot.commands.pathplanner.ConeCube;
import frc.team670.robot.commands.pathplanner.CubeEngage;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.LED;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.subsystems.arm.Arm;

/**
 * RobotContainer is where we put the high-level code for the robot.
 * It contains subsystems, OI devices, etc, and has required methods
 * (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {

    private final PowerDistribution pd = new PowerDistribution(1, ModuleType.kCTRE);

    private final Vision vision = new Vision(pd);
    private final DriveBase driveBase = new DriveBase(getDriverController());
    private final Arm arm = new Arm();
    private final Claw claw = new Claw(arm);
    private final LED led = new LED(RobotMap.LED_PORT,0,35);
    private static OI oi = new OI();
    
    private Notifier updateArbitraryFeedForward;

    public RobotContainer() {
        super();
        addSubsystem(driveBase, vision, arm, arm.getShoulder(), arm.getElbow(), arm.getWrist(), claw,led);
        oi.configureButtonBindings(driveBase, vision, arm, claw,led);

        for(MustangSubsystemBase subsystem : getSubsystems()) {
            subsystem.setDebugSubsystem(true);
        }
    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture();
        CvSink cvsink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Sebby Cam", 640, 480);
        driveBase.initVision(vision);

        updateArbitraryFeedForward = new Notifier(new Runnable() {
            public void run() {
                arm.updateArbitraryFeedForward();
            }
        });
        led.rainbow(false);

        updateArbitraryFeedForward.startPeriodic(0.01);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        SmartDashboard.putBoolean("match-started", true);     

        int selectedPath = (int) (SmartDashboard.getEntry("auton-chooser").getInteger(-1));
        System.out.println(selectedPath);
        // MustangCommand autonCommand;
        // switch (selectedPath) {
        //     case 0:
        //         autonCommand = new ConeCube(driveBase, claw, arm, "CableScore");
        //         break;
        //     case 1:
        //         autonCommand = new ConeCube(driveBase, claw, arm, "StationScore");
        //         break;
        //     case 2:
        //         autonCommand = new CubeEngage(driveBase, claw, arm, "CableEngage");
        //         break;
        //     case 3:
        //         autonCommand = new CubeEngage(driveBase, claw, arm, "StationEngage");
        //         break;
        //     case 4:
        //         autonCommand = new CenterEngage(driveBase, claw, arm, "CenterEngage");
        //         break;
        //     default:
        //         autonCommand = new CenterEngage(driveBase, claw, arm, "CenterEngage");
        // }
        // return autonCommand;
        return new AutonCalibration(driveBase, "Curve"); // TODO: use curve path after straight path

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
        arm.clearSetpoint();
    }

    @Override
    public void testInit() {}

    @Override
    public void disabled() {}
    

    @Override
    public void disabledPeriodic() {}

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
