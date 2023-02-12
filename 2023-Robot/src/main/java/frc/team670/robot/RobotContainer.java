/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import frc.team670.robot.commands.pathplanner.CubeEngage;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Notifier;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Vision;

import frc.team670.robot.subsystems.arm.Arm;

/**
 * RobotContainer is where we put the high-level code for the robot.
 * It contains subsystems, OI devices, etc, and has required methods
 * (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {

    private final PowerDistribution pd = new PowerDistribution(1, ModuleType.kCTRE);
    
    private final DriveBase driveBase = new DriveBase(getDriverController());
    private final Vision vision = new Vision(pd);
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();
    private static OI oi = new OI();
    
    private Notifier updateArbitraryFeedForwards;

    public RobotContainer() {
        super();
        addSubsystem(driveBase, vision, arm, arm.getShoulder(), arm.getElbow(), claw);
        oi.configureButtonBindings(driveBase, vision, arm, claw);
    }

    @Override
    public void robotInit() {
        updateArbitraryFeedForwards = new Notifier(new Runnable() {
            public void run() {
                arm.updateArbitraryFeedForwards();
            }
        });

        updateArbitraryFeedForwards.startPeriodic(0.01);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        return new CubeEngage(driveBase, claw, "RightCubeEngage");

        //return new ConeCube(driveBase, "LeftConeCube");

        // marker command (in path "StraightLine")

        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("event", new PrintOnStopPoint());

        // MustangFollowPathWithEvents command = new MustangFollowPathWithEvents(
        //     new MustangPPSwerveControllerCommand(
        //             trajectory,
        //             driveBase::getPose, 
        //             driveBase.getSwerveKinematics(),
        //             PID_x,
        //             PID_y,
        //             PID_theta,
        //             driveBase::setModuleStates,
        //             new Subsystem[] {driveBase}
        //             ),
        //     trajectory.getMarkers(),
        //     eventMap
        // );

        // return command;

        // the original command

        // return new MustangPPSwerveControllerCommand(
        //     trajectory,
        //     driveBase::getPose, 
        //     driveBase.getSwerveKinematics(),
        //     PID_x,
        //     PID_y,
        //     PID_theta,
        //     driveBase::setModuleStates,
        //     new Subsystem[] {driveBase}
        //     );

    }

    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void testInit() {
        // TODO Auto-generated method stub

    }

    @Override
    public void disabled() {
        // TODO Auto-generated method stub

    }

    @Override
    public void disabledPeriodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub

    }

    @Override
    public void autonomousPeriodic() {
        // TODO Auto-generated method stub

    }

    public MustangController getOperatorController() {
        // TODO Auto-generated method stub
        return null;
    }

    public MustangController getDriverController() {
        return OI.getDriverController();
    }

    public MustangController getBackupController() {
        // TODO Auto-generated method stub
        return null;
    }

}
