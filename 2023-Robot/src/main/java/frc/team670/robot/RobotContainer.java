/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import frc.team670.robot.commands.drivebase.MustangPPSwerveControllerCommand;
import frc.team670.robot.commands.pathplanner.MustangAutoBuilder;
import frc.team670.robot.commands.pathplanner.MustangFollowPathWithEvents;
import frc.team670.robot.commands.pathplanner.PrintOnStopPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
        //return new AutoLevel(driveBase);

        PathPlannerTrajectory trajectory = PathPlanner.loadPath("LeftConeCube", 1.0, 0.5);
        driveBase.resetOdometry(trajectory.getInitialHolonomicPose());
        
        PIDConstants PID_translation = new PIDConstants(1.0, 0, 0);
        PIDConstants PID_theta = new PIDConstants(1.0, 0, 0);
        // PID_theta.enableContinuousInput(-Math.PI, Math.PI);
        
        // auto builder - stop event command

        Map<String, Command> eventMap = new HashMap<>();
        eventMap.put("event", new PrintOnStopPoint());

        SwerveDriveKinematics driveBaseKinematics = driveBase.getSwerveKinematics();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            driveBase::getPose, 
            driveBase::resetOdometry,
            driveBaseKinematics, 
            PID_translation, 
            PID_theta,
            driveBase::setModuleStates, 
            eventMap,
            false,
            new Subsystem[] {driveBase}
        );

        MustangCommand fullAuto = (MustangAutoBuilder) autoBuilder.fullAuto(trajectory);

        return fullAuto;

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
