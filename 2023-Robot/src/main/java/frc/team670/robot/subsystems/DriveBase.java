// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.subsystems;


import frc.team670.mustanglib.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.XboxSwerveDrive;
import frc.team670.mustanglib.constants.*;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
// import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.constants.RobotConstants;

public class DriveBase extends SwerveDrive {
      /**
       * The maximum voltage that will be delivered to the drive motors. This can be reduced to cap
       * the robot's maximum speed. Typically, this is useful during initial testing of the robot.
       */
      public static final double MAX_VOLTAGE = 12.0;


      // The formula for calculating the theoretical maximum velocity is:
      // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
      // An example of this constant for a Mk4 L2 module with NEOs to drive is:
      // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
      // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
      /**
       * The maximum velocity of the robot in meters per second. This is a measure of how fast the
       * robot should be able to drive in a straight line.
       */
      public static final double MAX_VELOCITY_METERS_PER_SECOND =
                  5676.0 / 60.0 * SdsModuleConfigurations.MK4I_L1.getDriveReduction()
                              * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

      /**
       * The maximum angular velocity of the robot in radians per second. This is a measure of how
       * fast the robot can rotate in place.
       */
      public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
                  MAX_VELOCITY_METERS_PER_SECOND
                              / Math.hypot(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                          RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

      private MustangCommand defaultCommand;
      private MustangController mController;

      public DriveBase(MustangController mustangController, Vision vision) {
            super(new SwerveConfig(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS,
                        RobotConstants.DRIVETRAIN_WHEELBASE_METERS, MAX_VELOCITY_METERS_PER_SECOND,
                        MAX_VOLTAGE, RobotConstants.NAVX_PORT,
                        RobotConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        RobotConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                        RobotConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                        RobotConstants.FRONT_LEFT_MODULE_STEER_OFFSET,
                        RobotConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        RobotConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                        RobotConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                        RobotConstants.FRONT_RIGHT_MODULE_STEER_OFFSET,
                        RobotConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                        RobotConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                        RobotConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                        RobotConstants.BACK_LEFT_MODULE_STEER_OFFSET,
                        RobotConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        RobotConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                        RobotConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                        RobotConstants.BACK_RIGHT_MODULE_STEER_OFFSET)
                  );
            this.mController = mustangController;
      }


      /**
       * Makes the DriveBase's default command initialize teleop
       */
      public void initDefaultCommand() {  // TODO: switch to super class's init default command
            defaultCommand = new XboxSwerveDrive(this, mController, MAX_VELOCITY_METERS_PER_SECOND, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
            MustangScheduler.getInstance().setDefaultCommand(this, defaultCommand);
      }

      public void cancelDefaultCommand() {
            MustangScheduler.getInstance().cancel(defaultCommand);
      }

      @Override
      public HealthState checkHealth() {
            return HealthState.GREEN;
      }

      @Override
      public void debugSubsystem() {}

}
