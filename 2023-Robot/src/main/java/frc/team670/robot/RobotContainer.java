/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.commands.drivebase.AutoLevel;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.DriveBase;

/**
 * RobotContainer is where we put the high-level code for the robot.
 * It contains subsystems, OI devices, etc, and has required methods
 * (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {

    private final DriveBase driveBase = new DriveBase(getDriverController());

    private static OI oi = new OI();

    public RobotContainer() {
        super();
        addSubsystem(driveBase);
        oi.configureButtonBindings(driveBase);
    }

    @Override
    public void robotInit() {
        // TODO Auto-generated method stub
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        return new AutoLevel(driveBase);
        // PathPlannerTrajectory trajectory = PathPlanner.loadPath("s_curve", 0.5, 0.2);
        // driveBase.resetOdometry(trajectory.getInitialHolonomicPose());
        
        // PIDController PID_x = new PIDController(1.0, 0, 0);
        // PIDController PID_y = new PIDController(1.0, 0, 0);
        // PIDController PID_theta = new PIDController(1.0, 0, 0);
        // PID_theta.enableContinuousInput(-Math.PI, Math.PI);
        
        // return new MustangPPSwerveControllerCommand(
        //             trajectory,
        //             driveBase::getPose, 
        //             driveBase.getSwerveKinematics(),
        //             PID_x,
        //             PID_y,
        //             PID_theta,
        //             driveBase::setModuleStates,
        //             new Subsystem[] {driveBase}
        //             );
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

    @Override
    public MustangController getOperatorController() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public MustangController getDriverController() {
        return OI.getDriverController();
    }

    @Override
    public MustangController getBackupController() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
