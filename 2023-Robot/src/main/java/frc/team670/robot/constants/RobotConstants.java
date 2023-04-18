// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team670.robot.constants;

import static java.util.Map.entry;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import frc.team670.mustanglib.swervelib.Mk4ModuleConfiguration;
import frc.team670.mustanglib.swervelib.ModuleConfiguration;
import frc.team670.mustanglib.swervelib.SdsModuleConfigurations;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
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
    /**
     * Set your team number using the WPILib extension's "Set Team Number" action. 0) FACTORY RESET
     * ALL MOTOR CONTROLLERS 1) Set all of the *_ANGLE_OFFSET constants to -Math.toRadians(0.0). 2)
     * Deploy the code to your robot. Power cycle. NOTE: The robot isn't drivable quite yet, we
     * still have to setup the module offsets 3) Turn the robot on its side and align all the wheels
     * so they are facing in the forwards direction. NOTE: The wheels will be pointed forwards (not
     * backwards) when modules are turned so the large bevel gears are towards the LEFT side of the
     * robot. When aligning the wheels they must be as straight as possible. It is recommended to
     * use a long straight edge such as a piece of 2x1 in order to make the wheels straight. 4)
     * Record the angles of each module using the angle put onto Shuffleboard. The values are named
     * Front Left Module Angle, Front Right Module Angle, etc. 5) Set the values of the
     * *_ANGLE_OFFSET to -Math.toRadians(<the angle you recorded>) NOTE: All angles must be in
     * degrees. 6) Re-deploy and power cycle and try to drive the robot forwards. All the wheels
     * should stay parallel to each other. If not go back to step 3. 7) Make sure all the wheels are
     * spinning in the correct direction. If not, add 180 degrees to the offset of each wheel that
     * is spinning in the incorrect direction. i.e -Math.toRadians(<angle> + 180.0).
     */

    public static final String kSunTzuAddress = "00:80:2F:34:0B:07";
    public static final String kSkipperAddress = "00:80:2F:33:D0:46";
    public static final String kRobotAddress = getMACAddress();
    private static Map<String, Double> robotSpecificConstants = Map.ofEntries(entry(kSunTzuAddress,
            Map.ofEntries(entry("kBackRightModuleSteerOffsetRadians", -Math.toRadians(90.895)),
                    entry("kBackLeftModuleSteerOffsetRadians", -Math.toRadians(299.807)),
                    entry("kFrontRightModuleSteerOffsetRadians", -Math.toRadians(137.499)),
                    entry("kFrontLeftModuleSteerOffsetRadians", -Math.toRadians(318.604)),
                    entry("kShoulderAbsoluteEncoderVerticalOffsetRadians", 0.484233), // 0.47777
                    entry("kElbowAbsoluteEncoderVerticalOffsetRadians", 0.004892),
                    entry("kWristAbsoluteEncoderVerticalOffsetRadians", 0.337308),
                    entry("kShoulderGearRatio", 96.0), entry("kElbowGearRatio", 70.833333333333),
                    entry("kSwerveModuleConfig", 2.0), entry("kWristGearRatio", 125.0))),
            entry(kSkipperAddress,
                    Map.ofEntries(
                            entry("kBackRightModuleSteerOffsetRadians", -Math.toRadians(82.694)),
                            entry("kBackLeftModuleSteerOffsetRadians", -Math.toRadians(233.29)),
                            entry("kFrontRightModuleSteerOffsetRadians", -Math.toRadians(225.77)),
                            entry("kFrontLeftModuleSteerOffsetRadians", -Math.toRadians(112.53)),
                            entry("kShoulderAbsoluteEncoderVerticalOffsetRadians", 0.895),
                            entry("kElbowAbsoluteEncoderVerticalOffsetRadians", 0.588),
                            entry("kWristAbsoluteEncoderVerticalOffsetRadians", 0.918),
                            entry("kShoulderGearRatio", 75.0), entry("kElbowGearRatio", 90.0),
                            entry("kSwerveModuleConfig", 1.0), entry("kWristGearRatio", 125.0))))
            .get(kRobotAddress);

    public static final class DriveBase {
        public static final double kWidth = Units.inchesToMeters(36);
        public static double kClearance = Math.hypot(kWidth, kWidth) / 2 + 0.05;
        public static final double kTrackWidthMeters = 0.6096;
        public static final double kWheelBaseMeters = 0.6096;

        public static final ModuleConfiguration kModuleConfig =
                robotSpecificConstants.get("kSwerveModuleConfig") == 1.0
                        ? SdsModuleConfigurations.MK4I_L1
                        : SdsModuleConfigurations.MK4I_L2;

        public static final GearRatio kSwerveModuleGearRatio =
                robotSpecificConstants.get("kSwerveModuleConfig") == 1.0 ? GearRatio.L1
                        : GearRatio.L2;

        public static final int kFrontLeftModuleSteerMotorID = 20;
        public static final int kFrontLeftModuleDriveMotorID = 21;
        public static final int kFrontLeftModuleSteerEncoderID = 30;
        public static final double kFrontLeftModuleSteerOffsetRadians =
                robotSpecificConstants.get("kFrontLeftModuleSteerOffsetRadians");

        public static final int kFrontRightModuleSteerMotorID = 22;
        public static final int kFrontRightModuleDriveMotorID = 23;
        public static final int kFrontRightModuleSteerEncoderID = 32;
        public static final double kFrontRightModuleSteerOffsetRadians =
                robotSpecificConstants.get("kFrontRightModuleSteerOffsetRadians");

        public static final int kBackLeftModuleSteerMotorID = 26;
        public static final int kBackLeftModuleDriveMotorID = 27;
        public static final int kBackLeftModuleSteerEncoderID = 36;
        public static final double kBackLeftModuleSteerOffsetRadians =
                robotSpecificConstants.get("kBackLeftModuleSteerOffsetRadians");

        public static final int kBackRightModuleSteerMotorID = 24;
        public static final int kBackRightModuleDriveMotorID = 25;
        public static final int kBackRightModuleSteerEncoderID = 34;
        public static final double kBackRightModuleSteerOffsetRadians =
                robotSpecificConstants.get("kBackRightModuleSteerOffsetRadians");

        public final static SerialPort.Port kNAVXPort = SerialPort.Port.kMXP;
        public static final double kPitchOffset = 2;

        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 4;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 8;

        public static final double kMaxVoltage = 12.0;
        public static final double kMaxDriveCurrent = 45.0;
        public static final double kMaxSteerCurrent = 20.0;

        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        public static final double kMaxVelocityMetersPerSecond = 5676.0 / 60.0
                * kModuleConfig.getDriveReduction() * kModuleConfig.getWheelDiameter() * Math.PI;


        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond
                / Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0);

        public static final SwerveDrive.Config kConfig = new SwerveDrive.Config(kTrackWidthMeters,
                kWheelBaseMeters, kMaxVelocityMetersPerSecond, kMaxVoltage, kMaxDriveCurrent,
                kMaxSteerCurrent, kNAVXPort, kSwerveModuleGearRatio, kFrontLeftModuleDriveMotorID,
                kFrontLeftModuleSteerMotorID, kFrontLeftModuleSteerEncoderID,
                kFrontLeftModuleSteerOffsetRadians, kFrontRightModuleDriveMotorID,
                kFrontRightModuleSteerMotorID, kFrontRightModuleSteerEncoderID,
                kFrontRightModuleSteerOffsetRadians, kBackLeftModuleDriveMotorID,
                kBackLeftModuleSteerMotorID, kBackLeftModuleSteerEncoderID,
                kBackLeftModuleSteerOffsetRadians, kBackRightModuleDriveMotorID,
                kBackRightModuleSteerMotorID, kBackRightModuleSteerEncoderID,
                kBackRightModuleSteerOffsetRadians);

        public static final PIDConstants kAutonTranslationPID = new PIDConstants(4, 0, 0);
        public static final PIDConstants kAutonThetaPID = new PIDConstants(0.5, 0, 0);

        // PID controllers
        public static final PIDController xController = new PIDController(3, 0, 0);
        public static final PIDController yController = new PIDController(3, 0, 0);
        public static final PIDController thetaController = new PIDController(0.2, 0, 0);
        public static final PathConstraints kAutoPathConstraints = new PathConstraints(
                kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    }


    // vision
    public static final class Vision {
        public static final String[] kVisionCameraIDs = {"Arducam_B", "Arducam_C"};
        public static final Transform3d[] kCameraOffsets = {
                // Cam B - RIGHT
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(0.56), Units.inchesToMeters(-5.25),
                                Units.inchesToMeters(19 + 5)),
                        new Rotation3d(0, 0, Units.degreesToRadians(-45))),
                // Cam C - LEFT
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(0.56), Units.inchesToMeters(5.25),
                                Units.inchesToMeters(19 + 5)),
                        new Rotation3d(0, 0, Units.degreesToRadians(45)))};

        public static final double kLockedOnErrorX = 0.3;
        public static final double xLockedOnErrorY = 0.3;
        public static final double kLockedOnErrorDegrees = 10;
    }

    public static final class Arm {

        public static final class Shoulder {
            public static final int kLeaderMotorID = 2;
            public static final int kFollowerMotorID = 3;
            public static final int kAbsoluteEncoderID = 0;
            public static final double kAbsoluteEncoderVerticalOffsetRadians =
                    RobotConstants.robotSpecificConstants
                            .get("kShoulderAbsoluteEncoderVerticalOffsetRadians");
            public static final int kGearRatio =
                    (RobotConstants.robotSpecificConstants.get("kShoulderGearRatio")).intValue();
            public static final int kSoftLimitMin = 60;
            public static final int kSoftLimitMax = 300;
            public static final double kMaxOverrideDegrees = 15;
            public static final double kLengthInches = 26;
            public static final double kMassLb = 8.7;
            public static final double kMassDistribution = 0.356;
            public static final double kAllowedErrorDegrees = 0.2;
            public static final ArmSegment kSegment =
                    new ArmSegment(kLengthInches, kMassLb, kMassDistribution,
                            frc.team670.robot.subsystems.arm.Shoulder.SHOULDER_ARBITRARY_FF);

            public static final IdleMode kIdleMode = IdleMode.kBrake;

            public static final double kP = 0.0005;
            public static final double kI = 0;
            public static final double kD = 0.00005;
            public static final double kFF = 0.00017618;
            public static final double kIz = 0;


            public static final MotorConfig.Motor_Type kMotorType = MotorConfig.Motor_Type.NEO;
            public static final double kMaxOutput = 1;
            public static final double kMinOutput = -1;
            public static final double kMaxAcceleration = 2500;
            public static final float[] kSoftLimits =
                    new float[] {(float) Units.degreesToRotations(kSoftLimitMax),
                            (float) Units.degreesToRotations(kSoftLimitMin)};
            public static final int kContinuousCurrent = 20;
            public static final int kPeakCurrent = 60;

            public static final double kMaxRotatorRPM = 1500;
            public static final double kMinRotatorRPM = 0;

            public static final SparkMaxRotatingSubsystem.Config kConfig =
                    new SparkMaxRotatingSubsystem.Config(kLeaderMotorID, 0, kMotorType, kIdleMode,
                            kGearRatio, kP, kI, kD, kFF, kIz, kMaxOutput, kMinOutput,
                            kMaxRotatorRPM, kMinRotatorRPM, kMaxAcceleration, kAllowedErrorDegrees,
                            kSoftLimits, kContinuousCurrent, kPeakCurrent);

        }

        public static final class Elbow {
            public static final double kAbsoluteEncoderVerticalOffsetRadians =
                    RobotConstants.robotSpecificConstants
                            .get("kElbowAbsoluteEncoderVerticalOffsetRadians");
            public static final double kGearRatio =
                    RobotConstants.robotSpecificConstants.get("kElbowGearRatio");
            public static final int kSoftLimitMin = 20;
            public static final int kSoftLimitMax = 340;
            public static final int kMaxOverrideDegreees = 15;
            public static final double kAllowedErrorDegrees = 0.75;
            public static final double kLengthInches = 35;
            public static final double kMassLb = 5.5;
            public static final double kMassDistribution = 0.686;
            public static final ArmSegment kSegment =
                    new ArmSegment(kLengthInches, kMassLb, kMassDistribution, 0.5);

            public static final int kMotorID = 4;
            public static final IdleMode kIdleMode = IdleMode.kBrake;
            public static final int kAbsoluteEncoderID = 1;

            public static final double kP = 0.0007;
            public static final double kI = 0;
            public static final double kD = 0.00015;
            public static final double kFF = 0.000176;
            public static final double kIz = 0;


            public static final MotorConfig.Motor_Type kMotorType = MotorConfig.Motor_Type.NEO;
            public static final double kMaxOutput = 1;
            public static final double kMinOutput = -1;
            public static final double kMaxAcceleration = 4000;
            public static final float[] kSoftLimits =
                    new float[] {(float) Units.degreesToRotations(kSoftLimitMax),
                            (float) Units.degreesToRotations(kSoftLimitMin)};
            public static final int kContinuousCurrent = 20;
            public static final int kPeakCurrent = 60;

            public static final double kMaxRotatorRPM = 3000;
            public static final double kMinRotatorRPM = 0;

            public static final SparkMaxRotatingSubsystem.Config kConfig =
                    new SparkMaxRotatingSubsystem.Config(kMotorID, 0, kMotorType, kIdleMode,
                            kGearRatio, kP, kI, kD, kFF, kIz, kMaxOutput, kMinOutput,
                            kMaxRotatorRPM, kMinRotatorRPM, kMaxAcceleration, kAllowedErrorDegrees,
                            kSoftLimits, kContinuousCurrent, kPeakCurrent);
        }

        public static final class Wrist {

            // Wrist
            public static final double kAbsoluteEncoderVerticalOffsetRadians =
                    RobotConstants.robotSpecificConstants
                            .get("kWristAbsoluteEncoderVerticalOffsetRadians");
            public static final double kGearRatio =
                    (RobotConstants.robotSpecificConstants.get("kWristGearRatio")).intValue();
            public static final double kMaxOverrideDegrees = 90;
            public static final double kAllowedErrorDegrees = 1.5;
            public static final ArmSegment kWristSegment =
                    new ArmSegment(Claw.kLengthInches, Claw.kMassLB, Claw.kMassDistribution, 0);

            public static final int kMotorID = 5;
            public static final IdleMode kIdleMode = IdleMode.kBrake;
            public static final int kAbsoluteEncoderID = 2;

            public static final double kP = 0.00011;
            public static final double kI = 0;
            public static final double kD = 0.00015;
            public static final double kFF = 0.000176;
            public static final double kIz = 0;


            public static final MotorConfig.Motor_Type kMotorType = MotorConfig.Motor_Type.NEO_550;
            public static final double kMaxOutput = 1;
            public static final double kMinOutput = -1;
            public static final double kMaxAcceleration = 15000;
            public static final float[] kSoftLimits = null;
            public static final int kContinuousCurrent = 20;
            public static final int kPeakCurrent = 20;

            public static final double kMaxRotatorRPM = 6000;
            public static final double kMinRotatorRPM = 0;

            public static final SparkMaxRotatingSubsystem.Config kConfig =
                    new SparkMaxRotatingSubsystem.Config(kMotorID, 0, kMotorType, kIdleMode,
                            kGearRatio, kP, kI, kD, kFF, kIz, kMaxOutput, kMinOutput,
                            kMaxRotatorRPM, kMinRotatorRPM, kMaxAcceleration, kAllowedErrorDegrees,
                            kSoftLimits, kContinuousCurrent, kPeakCurrent);

        }

        public static final class Claw {

            public static final int kMotorID = 6;
            public static final double kLengthInches = 4;
            public static final double kMassLB = 6.5;
            public static final double kMassDistribution = 1;
            public static final double kRollingSpeed = 1.0;
            public static final double kEjectingSpeed = -0.60;
            public static final double kCurrentMax = 23.0;
            public static final double kIdleSpeed = 0.05;
            public static final int kEjectIterations = 30;
            public static final int kCurrentSpikeIterations = 25;
        }
    }

    public static final double kConeMassLb = 1.4;
    public static final int kLEDPort = 0;



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
