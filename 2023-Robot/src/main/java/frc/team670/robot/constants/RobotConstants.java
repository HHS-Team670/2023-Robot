// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.constants;

import static java.util.Map.entry;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import frc.team670.robot.subsystems.arm.ArmSegment;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotConstants extends RobotConstantsBase {

    public static final String MAC_ADDRESS = getMACAddress();

    /**
     * Set your team number using the WPILib extension's "Set Team Number" action. 0) FACTORY RESET
     * ALL MOTOR CONTROLLERS 1) Set all of the *_ANGLE_OFFSET constants to -Math.toRadians(0.0). 2)
     * Deploy the code to your robot. NOTE: The robot isn't drivable quite yet, we still have to
     * setup the module offsets 3) Turn the robot on its side and align all the wheels so they are
     * facing in the forwards direction. NOTE: The wheels will be pointed forwards (not backwards)
     * when modules are turned so the large bevel gears are towards the LEFT side of the robot. When
     * aligning the wheels they must be as straight as possible. It is recommended to use a long
     * strait edge such as a piece of 2x1 in order to make the wheels straight. 4) Record the angles
     * of each module using the angle put onto Shuffleboard. The values are named Front Left Module
     * Angle, Front Right Module Angle, etc. 5) Set the values of the *_ANGLE_OFFSET to
     * -Math.toRadians(<the angle you recorded>) NOTE: All angles must be in degrees. 6) Re-deploy
     * and try to drive the robot forwards. All the wheels should stay parallel to each other. If
     * not go back to step 3. 7) Make sure all the wheels are spinning in the correct direction. If
     * not, add 180 degrees to the offset of each wheel that is spinning in the incorrect direction.
     * i.e -Math.toRadians(<angle> + 180.0).
     */
    public static Map<String, Map<String, Double>> hardwareSpecificConstants = Map.ofEntries(
            entry("00:80:2F:34:0B:07", Map.ofEntries( // Mac address from 670_bench
                    entry("BACK_RIGHT_MODULE_STEER_OFFSET", -Math.toRadians(228.85)),
                    entry("BACK_LEFT_MODULE_STEER_OFFSET", -Math.toRadians(228.3)),
                    entry("FRONT_RIGHT_MODULE_STEER_OFFSET", -Math.toRadians(2.37)),
                    entry("FRONT_LEFT_MODULE_STEER_OFFSET", -Math.toRadians(30.2)),
                    entry("SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL", 0.957),
                    entry("ELBOW_ABSOLUTE_ENCODER_AT_VERTICAL", 0.494),
                    entry("WRIST_ABSOLUTE_ENCODER_AT_VERTICAL", 0.0),
                    entry("SHOULDER_GEAR_RATIO", 96.0), entry("ELBOW_GEAR_RATIO",
                            75.0),
                    entry("WRIST_GEAR_RATIO", 0.0))),
            entry("00:80:2F:33:D0:46", Map.ofEntries( // The mac address is from 670_2023
                    entry("BACK_RIGHT_MODULE_STEER_OFFSET", -Math.toRadians(82.694)),
                    entry("BACK_LEFT_MODULE_STEER_OFFSET", -Math.toRadians(233.29)),
                    entry("FRONT_RIGHT_MODULE_STEER_OFFSET", -Math.toRadians(225.77)),
                    entry("FRONT_LEFT_MODULE_STEER_OFFSET", -Math.toRadians(112.53)),
                    entry("SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL", 0.895),
                    entry("ELBOW_ABSOLUTE_ENCODER_AT_VERTICAL", 0.922),
                    entry("WRIST_ABSOLUTE_ENCODER_AT_VERTICAL", 0.918),
                    entry("SHOULDER_GEAR_RATIO", 75.0), entry("ELBOW_GEAR_RATIO", 90.0),
                    entry("WRIST_GEAR_RATIO", 125.0))),
            entry("00:80:2F:22:B4:F6", Map.ofEntries()) // The mac address is from 670_WCD (test
                                                        // bench)

    );

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */

    public static final double DRIVEBASE_WIDTH = Units.inchesToMeters(36);
    public static double DRIVEBASE_CLEARANCE =
            Math.hypot(DRIVEBASE_WIDTH, DRIVEBASE_WIDTH) / 2 + 0.05;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6096;

    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6096;

    public static final double LIMIT = 1.0;



    // Swerve Drivebase constants

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 25;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 24;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 34;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
            hardwareSpecificConstants.get(MAC_ADDRESS).get("BACK_RIGHT_MODULE_STEER_OFFSET");

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 27;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 26;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 36;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET =
            hardwareSpecificConstants.get(MAC_ADDRESS).get("BACK_LEFT_MODULE_STEER_OFFSET");

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 23;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 22;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 32;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
            hardwareSpecificConstants.get(MAC_ADDRESS).get("FRONT_RIGHT_MODULE_STEER_OFFSET");

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 21;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 20;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 30;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
            hardwareSpecificConstants.get(MAC_ADDRESS).get("FRONT_LEFT_MODULE_STEER_OFFSET");

    public final static SerialPort.Port NAVX_PORT = SerialPort.Port.kMXP;
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 16;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 16;

    // TODO: TUNE PID CONTROLLERS AND
    public static final PIDController xController = new PIDController(3, 0, 0);
    public static final PIDController yController = new PIDController(3, 0, 0);
    public static final PIDController thetaController = new PIDController(0.2, 0, 0);

    //Auton PID controllers
    public static final PIDConstants AUTON_TRANSLATION_CONTROLLER = new PIDConstants(4, 0, 0);
    public static final PIDConstants AUTON_THETA_CONTROLLER = new PIDConstants(0.3, 0, 0);

    // vision

    public static final String[] VISION_CAMERA_NAMES = {"Arducam_B", "Arducam_C"};
//    public static final String[] VISION_CAMERA_NAMES = {"Arducam_OV9281_USB_Camera", "Arducam_C"};
    public static final Transform3d[] CAMERA_OFFSETS = {
            // Cam B - LEFT
            new Transform3d(new Translation3d(Units.inchesToMeters(0.56),
                    Units.inchesToMeters(-5.25), Units.inchesToMeters(19 + 5)),
                    new Rotation3d(0, 0, 0)),
            // Cam C - RIGHT
            new Transform3d(new Translation3d(Units.inchesToMeters(0.56),
                    Units.inchesToMeters(5.25), Units.inchesToMeters(19 + 5)),
                    new Rotation3d(0, 0, 0))};

    public static final double LOCKED_ON_ERROR_X = 0.3; // TODO: test what angles are
                                                        // appropriate
                                                        // for grabbing
    public static final double LOCKED_ON_ERROR_Y = 0.3;
    public static final double LOCKED_ON_ERROR_DEGREES = 10;
    public static final double kTrackwidthMeters = 0.702;


    // Shoulder
    public static final double SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL =
            hardwareSpecificConstants.get(MAC_ADDRESS).get("SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL");
    public static final int SHOULDER_GEAR_RATIO =
            (hardwareSpecificConstants.get(MAC_ADDRESS).get("SHOULDER_GEAR_RATIO")).intValue();
    public static final int SHOULDER_SOFT_LIMIT_MIN = 60;
    public static final int SHOULDER_SOFT_LIMIT_MAX = 300;
    public static final double SHOULDER_ARBITRARY_FF = 0.5;
    public static final double SHOULDER_MAX_OVERRIDE_DEGREES = 15;

    public static final double SHOULDER_LENGTH_INCHES = 26;
    public static final double SHOULDER_TO_ELBOW_MASS_LB = 8.7;
    public static final double SHOULDER_MASS_DISTRIBUTION = 0.356; 
    public static final double SHOULDER_ALLOWED_ERR_DEG = 2;

    // Elbow
    public static final double ELBOW_ABSOLUTE_ENCODER_AT_VERTICAL =
            hardwareSpecificConstants.get(MAC_ADDRESS).get("ELBOW_ABSOLUTE_ENCODER_AT_VERTICAL");
    public static final int ELBOW_GEAR_RATIO =
            (hardwareSpecificConstants.get(MAC_ADDRESS).get("ELBOW_GEAR_RATIO")).intValue();;
    public static final int ELBOW_SOFT_LIMIT_MIN = 20;
    public static final int ELBOW_SOFT_LIMIT_MAX = 340;
    public static final int ELBOW_MAX_OVERRIDE_DEGREES  = 15;
    public static final double ELBOW_ALLOWED_ERR_DEG = 2;
    // public static final double ELBOW_ARBITRARY_FF = 0.0;
    public static final double ELBOW_ARBITRARY_FF = 0.5;
    public static final double ELBOW_LENGTH_INCHES = 35;
    public static final double ELBOW_TO_CLAW_MASS_LB = 5.5; // TODO: set
    public static final double ELBOW_MASS_DISTRIBUTION = 0.686; // TODO: set

    // Wrist
    public static final double WRIST_ABSOLUTE_ENCODER_AT_VERTICAL =
            hardwareSpecificConstants.get(MAC_ADDRESS).get("WRIST_ABSOLUTE_ENCODER_AT_VERTICAL");
    public static final double WRIST_GEAR_RATIO =
            (hardwareSpecificConstants.get(MAC_ADDRESS).get("WRIST_GEAR_RATIO")).intValue();;
    public static final double WRIST_ARBITRARY_FF = 0;
    public static final double WRIST_MAX_OVERRIDE_DEGREES = 90;
    public static final double WRIST_ALLOWED_ERR_DEG = 2;

    public static final double CLAW_LENGTH_INCHES = 4;
    public static final double CLAW_MASS_LB = 6.5;
    public static final double CLAW_MASS_DISTRIBUTION = 1;

    public static final double CONE_MASS_LB = 1.4;

    // Gravity-cancelling constants
    public static final ArmSegment SHOULDER_SEGMENT = new ArmSegment(SHOULDER_LENGTH_INCHES,
            SHOULDER_TO_ELBOW_MASS_LB, SHOULDER_MASS_DISTRIBUTION, SHOULDER_ARBITRARY_FF);
    public static final ArmSegment ELBOW_SEGMENT = new ArmSegment(ELBOW_LENGTH_INCHES,
            ELBOW_TO_CLAW_MASS_LB, ELBOW_MASS_DISTRIBUTION, ELBOW_ARBITRARY_FF);
    public static final ArmSegment WRIST_SEGMENT = new ArmSegment(CLAW_LENGTH_INCHES, CLAW_MASS_LB,
            CLAW_MASS_DISTRIBUTION, WRIST_ARBITRARY_FF);



    // claw constants
    public static final double CLAW_ROLLING_SPEED = 1.0;
    public static final double CLAW_EJECTING_SPEED = -0.75;
    public static final double CLAW_CURRENT_MAX = 23.0;
    public static final double CLAW_IDLE_SPEED = 0.05;
    public static final int CLAW_EJECT_ITERATIONS = 25;
    public static final int CLAW_CURRENT_SPIKE_ITERATIONS = 5;

    public static final int kTimeoutMs = 0;
    public static final double leftKsVolts = 0.4;
    public static final double leftKvVoltSecondsPerMeter = 2.1;
    public static final double leftKaVoltSecondsSquaredPerMeter = 0.15;
    public static final double rightKsVolts = leftKsVolts;
    public static final double rightKvVoltSecondsPerMeter = leftKvVoltSecondsPerMeter;
    public static final double rightKaVoltSecondsSquaredPerMeter = leftKaVoltSecondsSquaredPerMeter;

    //Auto contraints for vision
    public static final PathConstraints kAutoPathConstraints =
            new PathConstraints(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);


    /**
     * This is code from Poofs 2022
     * 
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
                            ret.append(String.format("%02X%s", mac[i],
                                    (i < mac.length - 1) ? ":" : ""));
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
        System.out.println("\n\nMAC ADDRESS NOTHING");
        return "";
    }

}
