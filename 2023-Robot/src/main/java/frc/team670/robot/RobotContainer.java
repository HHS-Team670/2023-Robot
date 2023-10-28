/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.OI;
import frc.team670.robot.subsystems.LED;

/**
 * RobotContainer is where we put the high-level code for the robot. It contains
 * subsystems, OI
 * devices, etc, and has required methods (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {

    private final LED mLed = LED.getInstance();




   

    public RobotContainer() {
        super();
        addSubsystem( mLed);
        OI.configureButtonBindings();

        for (MustangSubsystemBase subsystem : getSubsystems()) {
            subsystem.setDebugSubsystem(true);
        }

       
        

    }

    @Override
    public void robotInit() {
        CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kYUYV, 160, 120, 30);

        // mDriveBase.initVision(mVision);
    
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public MustangCommand getAutonomousCommand() {
        

        int selectedPath=0; // This value is typically used in our auton chooser and  sets both the auton path and the default led pattern
        // Note the below line doesn't do much because it is repeatdly called in disabled periodic. This can probably be removed with testing
       // if this still exists make sure to chance the selected path value in  disabled periodic
        mLed.updateAutonPathColor(selectedPath);
        // mLed.solidhsv(LEDColor.SEXY_PURPLE);// I really don't know why we do this beacuse it litterally won't do anything
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
        
        int selectedPath = 0;
        // The `mLed.updateAutonPathColor(selectedPath)` method is updating the color of the LED
        // subsystem based on the selected autonomous path. The `selectedPath` variable represents the
        // chosen autonomous path, and the `updateAutonPathColor()` method in the `LED` subsystem is
        // responsible for changing the LED color accordingly.
        

        mLed.updateAutonPathColor(selectedPath);
    }

    @Override
    public void periodic() {
       

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
