// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.constants;

import static java.util.Map.entry;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team670.mustanglib.constants.RobotConstantsBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotConstants extends RobotConstantsBase {

    public static final String MAC_ADDRESS = getMACAddress();

    // Set your team number using the WPILib extension's "Set Team Number" action.
    // 1) Set all of the *_ANGLE_OFFSET constants to -Math.toRadians(0.0).
    // 2) Deploy the code to your robot.
    //      NOTE: The robot isn't drivable quite yet, we still have to setup the module offsets
    // 3) Turn the robot on its side and align all the wheels so they are facing in the forwards direction.
    //      NOTE: The wheels will be pointed forwards (not backwards) when modules are turned so the large bevel gears are towards the LEFT side of the robot. When aligning the wheels they must be as straight as possible. It is recommended to use a long strait edge such as a piece of 2x1 in order to make the wheels straight.
    // 4) Record the angles of each module using the angle put onto Shuffleboard. The values are named Front Left Module Angle, Front Right Module Angle, etc.
    // 5) Set the values of the *_ANGLE_OFFSET to -Math.toRadians(<the angle you recorded>)
    //      NOTE: All angles must be in degrees.
    // 6) Re-deploy and try to drive the robot forwards. All the wheels should stay parallel to each other. If not go back to step 3.
    // 7) Make sure all the wheels are spinning in the correct direction. If not, add 180 degrees to the offset of each wheel that is spinning in the incorrect direction. i.e -Math.toRadians(<angle> + 180.0).
    public static Map<String, Map<String, Double>> hardwareSpecificConstants = Map.ofEntries(
        entry("00:80:2F:34:0B:07", Map.ofEntries( //Mac address from 670_bench
            entry("BACK_RIGHT_MODULE_STEER_OFFSET", -Math.toRadians(228.85)),
            entry("BACK_LEFT_MODULE_STEER_OFFSET",-Math.toRadians(228.3)),
            entry("FRONT_RIGHT_MODULE_STEER_OFFSET", -Math.toRadians(2.37)),
            entry("FRONT_LEFT_MODULE_STEER_OFFSET", -Math.toRadians(30.2))
        )),
        entry("00:80:2F:24:4A:34", Map.ofEntries( //The mac address is from 670_MadMax
            entry("BACK_RIGHT_MODULE_STEER_OFFSET", -Math.toRadians(292.5)),
            entry("BACK_LEFT_MODULE_STEER_OFFSET",-Math.toRadians(232.91)),
            entry("FRONT_RIGHT_MODULE_STEER_OFFSET", -Math.toRadians(352.35)),
            entry("FRONT_LEFT_MODULE_STEER_OFFSET", -Math.toRadians(136.67))    
        ))

    );

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6096;

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6096;

    public static final double LIMIT = 1.0;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 25; //  Set back right module drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 24; //  Set back right module steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 34; //  Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = hardwareSpecificConstants.get(MAC_ADDRESS).get("BACK_RIGHT_MODULE_STEER_OFFSET"); //  Measure and set back right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 27; //  Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 26; //  Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 36; //  Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = hardwareSpecificConstants.get(MAC_ADDRESS).get("BACK_LEFT_MODULE_STEER_OFFSET"); //  Measure and set back left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 23; //  Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 22; //  Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 32; //  Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = hardwareSpecificConstants.get(MAC_ADDRESS).get("FRONT_RIGHT_MODULE_STEER_OFFSET"); //  Measure and set back left steer offset
 //  Measure and set front right steer offset
    
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 21; //  Set front left drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20; //  Set front left steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 30; //  Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = hardwareSpecificConstants.get(MAC_ADDRESS).get("FRONT_LEFT_MODULE_STEER_OFFSET"); //   Measure and set front left steer offset

    public final static SerialPort.Port NAVX_PORT = SerialPort.Port.kMXP;
    
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;
    
    // vision
    public static final String VISION_CAMERA_NAME = "Microsoft_LifeCam_HD-3000"; // TODO: change to global shutter camera name
    public static final Transform2d CAMERA_OFFSET = 
        new Transform2d(new Translation2d(0, 0), new Rotation2d(0));    // TODO: changed when camera actually mounted, may need to change based on robot
    public static final Transform2d GRID_TO_TARGET_OFFSET = 
        new Transform2d(new Translation2d(1, 0), new Rotation2d(0));    // TODO: check game manual for actual specs
    
    
        //Everything below is copied from 2022 robot
    public static final double kTrackwidthMeters = 0.702;

    public static final int kTimeoutMs = 0;
    public static final double leftKsVolts = 0.4;
    public static final double leftKvVoltSecondsPerMeter = 2.1;
    public static final double leftKaVoltSecondsSquaredPerMeter = 0.15;
    public static final double rightKsVolts = leftKsVolts;
    public static final double rightKvVoltSecondsPerMeter = leftKvVoltSecondsPerMeter;
    public static final double rightKaVoltSecondsSquaredPerMeter = leftKaVoltSecondsSquaredPerMeter;
    public static final DifferentialDriveKinematics kDriveKinematics = 
    new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final DifferentialDriveKinematicsConstraint kAutoPathConstraints = 
            new DifferentialDriveKinematicsConstraint(kDriveKinematics, kMaxSpeedMetersPerSecond);

    /**
     * This is code from Poofs 2022
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                System.out.println("NIS: " + nis.getDisplayName());
                if (nis != null && "eth0".equals(nis.getDisplayName())) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                        }
                        String addr = ret.toString();
                        System.out.println("NIS " + nis.getDisplayName() + " addr: " + addr);
                        return addr;
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Skipping adaptor: " + nis.getDisplayName());
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}
