/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotContainerBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.LEDSubsystem.LEDColor;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.OI;


import frc.team670.robot.subsystems.LED;
import frc.team670.mustanglib.subsystems.LEDSubsystem.LEDColor;


/**
 * RobotContainer is where we put the high-level code for the robot. It contains
 * subsystems, OI
 * devices, etc, and has required methods (autonomousInit, periodic, etc)
 */

public class RobotContainer extends RobotContainerBase {
    private final LED mLed = LED.getInstance();

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
        mLed.rainbow();
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
        SmartDashboard.putNumber("Led Style", 0);
        switch((int)SmartDashboard.getNumber("Led Style", 0)){
            case 1:
                mLed.solidhsv(LEDColor.BLUE);
                break;
            case 2:
                // mLed.blinking(LEDColor.BLUE);
            default:
                mLed.rainbow();
                break;
        }
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
