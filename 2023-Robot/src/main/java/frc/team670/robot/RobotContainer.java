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
    private final LED mLed = LED.getInstance();


    private final String kMatchStartedString = "match-started";
    private final String kAutonChooserString = "auton-chooser";

    public RobotContainer() {
        super();
        addSubsystem(mLed);
        OI.configureButtonBindings();

        for (MustangSubsystemBase subsystem : getSubsystems()) {
            subsystem.setDebugSubsystem(true);
        }

      
    }

    @Override
    public void robotInit() {
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        return null;
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void teleopInit() {
        
    }

    @Override
    public void testInit() {
    }

    @Override
    public void disabled() {
    }

    @Override
    public void disabledPeriodic() {
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
    }

    @Override
    public void teleopPeriodic() {
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
