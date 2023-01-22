// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.constants;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.Map;

import static java.util.Map.entry; 

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

    public static Map<String, Map<String, Double>> hardwareSpecificConstants = Map.ofEntries(
        entry("00:80:2F:34:0B:07", Map.ofEntries( //Mac address from BLUE
            entry("BACK_RIGHT_MODULE_STEER_OFFSET", 0.0),
            entry("BACK_LEFT_MODULE_STEER_OFFSET",0.0),
            entry("FRONT_RIGHT_MODULE_STEER_OFFSET", 0.0),
            entry("FRONT_LEFT_MODULE_STEER_OFFSET", 0.0)
        )),
        entry("00:80:2F:24:4A:34", Map.ofEntries( //The mac address is from RED 
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

    //Everything below is copied from 2022 robot

    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

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
