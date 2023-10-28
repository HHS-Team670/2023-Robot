// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.constants;

import static java.util.Map.entry;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team670.mustanglib.RobotConstantsBase;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase.TagCountDeviation;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase.UnitDeviationParams;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import frc.team670.mustanglib.swervelib.ModuleConfiguration;
import frc.team670.mustanglib.swervelib.SdsModuleConfigurations;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotConstants extends RobotConstantsBase {
    /**
     * Set your team number using the WPILib extension's "Set Team Number" action.
     * 0) FACTORY RESET
     * ALL MOTOR CONTROLLERS 1) Set all of the *_ANGLE_OFFSET constants to
     * -Math.toRadians(0.0). 2)
     * Deploy the code to your robot. Power cycle. NOTE: The robot isn't drivable
     * quite yet, we
     * still have to setup the module offsets 3) Turn the robot on its side and
     * align all the wheels
     * so they are facing in the forwards direction. NOTE: The wheels will be
     * pointed forwards (not
     * backwards) when modules are turned so the large bevel gears are towards the
     * LEFT side of the
     * robot. When aligning the wheels they must be as straight as possible. It is
     * recommended to
     * use a long straight edge such as a piece of 2x1 in order to make the wheels
     * straight. 4)
     * Record the angles of each module using the angle put onto Shuffleboard. The
     * values are named
     * Front Left Module Angle, Front Right Module Angle, etc. 5) Set the values of
     * the
     * *_ANGLE_OFFSET to -Math.toRadians(<the angle you recorded>) NOTE: All angles
     * must be in
     * degrees. 6) Re-deploy and power cycle and try to drive the robot forwards.
     * All the wheels
     * should stay parallel to each other. If not go back to step 3. 7) Make sure
     * all the wheels are
     * spinning in the correct direction. If not, add 180 degrees to the offset of
     * each wheel that
     * is spinning in the incorrect direction. i.e -Math.toRadians(<angle> + 180.0).
     */




  
    public static final class Led {
        public static final int kPort = 0;
        public static final int kStartIndex = 0;
        public static final int kEndindex = 61;
    }
    

}
