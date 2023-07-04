// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.XboxSwerveDrive;
import frc.team670.mustanglib.swervelib.SwerveDrive;
import frc.team670.mustanglib.swervelib.SwerveModule;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.RobotConstants;

public class DriveBase extends SwerveDrive {
    private static DriveBase mInstance;
    private MustangCommand defaultCommand;
    private MustangController mController;
    public static synchronized DriveBase getInstance() {
        mInstance = mInstance == null ? new DriveBase() : mInstance;
        return mInstance;
    }

    public DriveBase() {
        super(RobotConstants.DriveBase.kConfig);
    }


     public void initDefaultCommand() { // TODO: switch to super class's init default command
            // defaultCommand = new XboxSwerveDrive(this, mController,
            // MAX_VELOCITY_METERS_PER_SECOND,
            // MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
            defaultCommand = new XboxSwerveDrive(this, mController, RobotConstants.DriveBase.kMaxVelocityMetersPerSecond, RobotConstants.DriveBase.kMaxAngularVelocityRadiansPerSecond);
            MustangScheduler.getInstance().setDefaultCommand(this, defaultCommand);
      }

    public void mustangPeriodic() {
        super.mustangPeriodic();
        SmartDashboard.putNumber("pitch", getPitchDegrees());
    }

    @Override
    public HealthState checkHealth() {
        for (SwerveModule curr : getModules()) {
            CANSparkMax motor = (CANSparkMax) curr.getDriveMotor();
            if (motor.getLastError() != REVLibError.kOk) {
                SmartDashboard.putString("Swerve Module " + motor.getDeviceId() + " ERROR:",
                        motor.getLastError().toString());
                return HealthState.RED;
            }
        }
        return HealthState.GREEN;
    }

    @Override
    public void debugSubsystem() {}

}
