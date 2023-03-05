/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.drivebase.SwerveDriveParkCommand;
import frc.team670.robot.commands.pathplanner.ConeCube;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.LED;

/**
 * RobotContainer is where we put the high-level code for the robot.
 * It contains subsystems, OI devices, etc, and has required methods
 * (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {

    private final PowerDistribution pd = new PowerDistribution(1, ModuleType.kCTRE);

    // private final Vision vision = new Vision(pd);
    // private final DriveBase driveBase = new DriveBase(getDriverController());
    // private final Arm arm = new Arm();
    // private final Claw claw = new Claw(arm);
    private final LED led = new LED(3, 0, 225);
    private static OI oi = new OI();

    private Notifier updateArbitraryFeedForward;

    public RobotContainer() {
        super();
        addSubsystem(led);
        oi.configureButtonBindings(led);

        for (MustangSubsystemBase subsystem : getSubsystems()) {
            subsystem.setDebugSubsystem(true);
        }
    }

    @Override
    public void robotInit() {
        // driveBase.initVision(vision);

        // updateArbitraryFeedForward = new Notifier(new Runnable() {
        //     public void run() {
        //         arm.updateArbitraryFeedForward();
        //     // }
        // });
        
        // updateArbitraryFeedForward.startPeriodic(0.01);
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        // return new AutonCalibration(driveBase, "StraightLine"); TODO: use curve path
        // after straight path
        // return new ConeCube(driveBase, claw, arm, "CableScore");
        // return new ConeCube(driveBase, claw, arm, "RightConeCube");
        // return new NonPidAutoLevel(driveBase, true);
        return null;
    }

    @Override
    public void autonomousInit() {
        // arm.setStateToStarting();
    }

    @Override
    public void teleopInit() {
        // arm.setStateToStarting();
        // arm.clearSetpoint();
        led.solidrgb(LEDColor.GREEN);
        
    }

    @Override
    public void testInit() {
    }

    @Override
    public void disabled() {
        led.rainbow(false);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void periodic() {
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
