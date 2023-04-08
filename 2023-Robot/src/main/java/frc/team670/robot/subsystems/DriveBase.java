// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.XboxSwerveDrive;
import frc.team670.mustanglib.constants.SwerveConfig;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.swervelib.SdsModuleConfigurations;
import frc.team670.mustanglib.swervelib.SwerveModule;
import frc.team670.mustanglib.swervelib.pathplanner.MustangPPSwerveControllerCommand;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.RobotConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends SwerveDrive {


    private MustangCommand defaultCommand;
    private MustangController mController;

    public DriveBase(MustangController mustangController) {
        super(RobotConstants.DriveBase.kConfig);
        this.mController = mustangController;
    }

    /**
     * Makes the DriveBase's default command initialize teleop
     */
    public void initDefaultCommand() { // TODO: switch to super class's init default command
        // defaultCommand = new XboxSwerveDrive(this, mController,
        // MAX_VELOCITY_METERS_PER_SECOND,
        // MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        defaultCommand = new XboxSwerveDrive(this, mController,
                RobotConstants.DriveBase.kMaxVelocityMetersPerSecond,
                RobotConstants.DriveBase.kMaxAngularVelocityRadiansPerSecond);
        MustangScheduler.getInstance().setDefaultCommand(this, defaultCommand);
    }

    public void cancelDefaultCommand() {
        MustangScheduler.getInstance().cancel(defaultCommand);
    }

    public MustangCommand getDefaultMustangCommand() {
        return defaultCommand;
    }

    public void mustangPeriodic() {
        super.mustangPeriodic();
        SmartDashboard.putNumber("pitch", getPitch());
    }

    // @Override
    // public void periodic() {
    // return;
    // }

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
    public void debugSubsystem() {

    }

    public MustangPPSwerveControllerCommand getFollowTrajectoryCommand(PathPlannerTrajectory traj) {
        setSwerveControllerCommand(new MustangPPSwerveControllerCommand(traj, this::getPose,
                getSwerveKinematics(), RobotConstants.DriveBase.xController,
                RobotConstants.DriveBase.yController, RobotConstants.DriveBase.thetaController,
                this::setModuleStates, new Subsystem[] {this}));
        return getSwerveControllerCommand();

    }

}
