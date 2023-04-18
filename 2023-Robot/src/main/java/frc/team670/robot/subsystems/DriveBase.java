// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.swervelib.SwerveModule;
import frc.team670.robot.constants.RobotConstants;

public class DriveBase extends SwerveDrive {
    private static DriveBase mInstance;

    public static synchronized DriveBase getInstance() {
        return mInstance == null ? new DriveBase() : mInstance;
    }

    public DriveBase() {
        super(RobotConstants.DriveBase.kConfig);
    }

    public void mustangPeriodic() {
        super.mustangPeriodic();
        SmartDashboard.putNumber("pitch", getPitch());
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
