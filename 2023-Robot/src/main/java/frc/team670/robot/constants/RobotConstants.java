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
import frc.team670.robot.subsystems.arm.ArmSegment;

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

    public static final String kSunTzuAddress = "00:80:2F:34:0B:07";
    public static final String kSkipperAddress = "00:80:2F:33:D0:46";
    public static final String kRobotAddress = getMACAddress();
    
    private static  Map<String, Double> robotSpecificConstants = Map.ofEntries(
        entry(kSunTzuAddress, Map.ofEntries(
                    entry("kBackRightModuleSteerOffsetRadians", -Math.toRadians(90.895)),
                    entry("kBackLeftModuleSteerOffsetRadians", -Math.toRadians(299.807)),
                    entry("kFrontRightModuleSteerOffsetRadians", -Math.toRadians(137.499)),
                    entry("kFrontLeftModuleSteerOffsetRadians", -Math.toRadians(318.604)),
                    entry("kShoulderAbsoluteEncoderVerticalOffset", 0.484233), // 0.47777
                    entry("kElbowAbsoluteEncoderVerticalOffset", 0.004892),
                    entry("kWristAbsoluteEncoderVerticalOffset", 0.420540), // 0.177702
                    entry("kShoulderGearRatio", 96.0), entry("kElbowGearRatio", 70.833333333333),
                    entry("kSwerveModuleConfig", 2.0), entry("kWristGearRatio", 45.0), 
                    entry("kDeployerGearRatio",25.0), entry("kDeployerAbsoluteEncoderVerticalOffset",0.792352))),
            entry(kSkipperAddress,
                    Map.ofEntries(
                            entry("kBackRightModuleSteerOffsetRadians", -Math.toRadians(82.694)),
                            entry("kBackLeftModuleSteerOffsetRadians", -Math.toRadians(233.29)),
                            entry("kFrontRightModuleSteerOffsetRadians", -Math.toRadians(225.77)),
                            entry("kFrontLeftModuleSteerOffsetRadians", -Math.toRadians(112.53)),
                            entry("kShoulderAbsoluteEncoderVerticalOffset", 0.895),
                            entry("kElbowAbsoluteEncoderVerticalOffset", 0.588),
                            entry("kWristAbsoluteEncoderVerticalOffset", 0.918),
                            entry("kShoulderGearRatio", 75.0), entry("kElbowGearRatio", 90.0),
                            entry("kSwerveModuleConfig", 1.0), entry("kWristGearRatio", 125.0),
                            entry("kDeployerGearRatio",25.0),entry("kAbsoluteEncoderPostionAtFlipoutZero",-0.25), entry("kAbsoluteEncoderPostionAtFlipoutMax", 0.07))))
            .get(kRobotAddress);

    public static final class DriveBase extends SwerveDriveBase {
       //Remember

       public static final double kWidth = Units.inchesToMeters(36);
       public static double kClearance = Math.hypot(kWidth, kWidth) / 2 + 0.05;
       public static final double kTrackWidthMeters = 0.6096;
       public static final double kWheelBaseMeters = 0.6096;
       
       public final static SerialPort.Port kNAVXPort = SerialPort.Port.kMXP;

       public static final double kMaxSpeedMetersPerSecond = 2;// 1. Not constant between auton path, 2. Robot specific
       public static final double kMaxAccelerationMetersPerSecondSquared = 1;

       public static final PathConstraints kAutoPathConstraints = new PathConstraints(
        kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

        public static final ModuleConfiguration kModuleConfig = robotSpecificConstants.get("kSwerveModuleConfig") == 1.0
                ? SdsModuleConfigurations.MK4I_L1
                : SdsModuleConfigurations.MK4I_L2;

        public static final GearRatio kSwerveModuleGearRatio = robotSpecificConstants.get("kSwerveModuleConfig") == 1.0
                ? GearRatio.L1
                : GearRatio.L2;
        public static final double kFrontLeftModuleSteerOffsetRadians = robotSpecificConstants
                .get("kFrontLeftModuleSteerOffsetRadians");

        public static final double kFrontRightModuleSteerOffsetRadians = robotSpecificConstants
                .get("kFrontRightModuleSteerOffsetRadians");

        public static final double kBackLeftModuleSteerOffsetRadians = robotSpecificConstants
                .get("kBackLeftModuleSteerOffsetRadians");
                
        public static final double kBackRightModuleSteerOffsetRadians = robotSpecificConstants
                .get("kBackRightModuleSteerOffsetRadians");
      
        
        

        public static final double kMaxVelocityMetersPerSecond = 5676.0 / 60.0
                * kModuleConfig.getDriveReduction() * kModuleConfig.getWheelDiameter() * Math.PI;

        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond
                / Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0);

        public static final SwerveDrive.Config kConfig = new SwerveDrive.Config(kTrackWidthMeters,
                kWheelBaseMeters, kMaxVelocityMetersPerSecond,kMaxAngularVelocityRadiansPerSecond, kMaxVoltage, kMaxDriveCurrent,
                kMaxSteerCurrent, kNAVXPort, kSwerveModuleGearRatio, kFrontLeftModuleDriveMotorID,
                kFrontLeftModuleSteerMotorID, kFrontLeftModuleSteerEncoderID,
                kFrontLeftModuleSteerOffsetRadians, kFrontRightModuleDriveMotorID,
                kFrontRightModuleSteerMotorID, kFrontRightModuleSteerEncoderID,
                kFrontRightModuleSteerOffsetRadians, kBackLeftModuleDriveMotorID,
                kBackLeftModuleSteerMotorID, kBackLeftModuleSteerEncoderID,
                kBackLeftModuleSteerOffsetRadians, kBackRightModuleDriveMotorID,
                kBackRightModuleSteerMotorID, kBackRightModuleSteerEncoderID,
                kBackRightModuleSteerOffsetRadians);


    }

    // vision
    public static final class Vision {
        public static final String[] kVisionCameraIDs = {"Arducam_B","Arducam_C" };
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

        public static final double kPoseAmbiguityCutOff = 0.05;
        public static final List<Set<Integer>> kPossibleFrameFIDCombos = List.of(Set.of(1, 2, 3, 4),
                Set.of(5, 6, 7, 8));

        public static final int kMaxFrameFIDs = 4;
        public static final Map<Integer, TagCountDeviation> kVisionStdFromTagsSeen = Map.ofEntries(
            Map.entry(1, new TagCountDeviation(
                new UnitDeviationParams(.25, .4, .9),
                new UnitDeviationParams(.35, .5, 1.2),
                new UnitDeviationParams(.5, .7, 1.5))),
            Map.entry(2, new TagCountDeviation(
                new UnitDeviationParams(.35, .1, .4), new UnitDeviationParams(.5, .7, 1.5))),
            Map.entry(3, new TagCountDeviation(
                new UnitDeviationParams(.25, .07, .25), new UnitDeviationParams(.15, 1, 1.5)))
        );
        
        public static final AprilTagFieldLayout kFieldLayout = FieldConstants.getFieldLayout(FieldConstants.aprilTags);
        public static final VisionSubsystemBase.Config kConfig = new VisionSubsystemBase.Config(kFieldLayout, kVisionCameraIDs, kCameraOffsets, kPoseAmbiguityCutOff, kMaxFrameFIDs, kPossibleFrameFIDCombos, kVisionStdFromTagsSeen);
    }

    public static final class Arm {

        public static final class Shoulder {
            public static final int kLeaderMotorID = 2;
            public static final int kFollowerMotorID = 3;
            public static final int kAbsoluteEncoderID = 0;
            public static final double kAbsoluteEncoderVerticalOffset = RobotConstants.robotSpecificConstants
                    .get("kShoulderAbsoluteEncoderVerticalOffset");
            public static final int kGearRatio = (RobotConstants.robotSpecificConstants.get("kShoulderGearRatio"))
                    .intValue();
            public static final int kSoftLimitMin = 60;
            public static final int kSoftLimitMax = 300;
            public static final double kMaxOverrideDegrees = 15;
            public static final double kLengthInches = 26;
            public static final double kMassLb = 8.7;
            public static final double kMassDistribution = 0.356;
            public static final double kAllowedErrorDegrees= 0.2;
            public static final double kAllowedErrorRotations = kGearRatio*kAllowedErrorDegrees/360;
            
            public static final ArmSegment kSegment = new ArmSegment(kLengthInches, kMassLb, kMassDistribution,
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
            //public static final float[] kSoftLimits = new float[] { (float) Units.degreesToRotations(kSoftLimitMax), (float) Units.degreesToRotations(kSoftLimitMin) };
            public static final float[] kSoftLimits = null;
            public static final int kContinuousCurrent = 20;
            public static final int kPeakCurrent = 60;

            public static final double kMaxRotatorRPM = 1500;
            public static final double kMinRotatorRPM = 0;

            public static final SparkMaxRotatingSubsystem.Config kConfig = new SparkMaxRotatingSubsystem.Config(
                    kLeaderMotorID, 0, kMotorType, kIdleMode,
                    kGearRatio, kP, kI, kD, kFF, kIz, kMaxOutput, kMinOutput,
                    kMaxRotatorRPM, kMinRotatorRPM, kMaxAcceleration, kAllowedErrorRotations,
                    kSoftLimits, kContinuousCurrent, kPeakCurrent);

        }

        public static final class Elbow {
            public static final double kAbsoluteEncoderVerticalOffset = RobotConstants.robotSpecificConstants
                    .get("kElbowAbsoluteEncoderVerticalOffset");
            public static final double kGearRatio = RobotConstants.robotSpecificConstants.get("kElbowGearRatio");
            public static final int kSoftLimitMin = 20;
            public static final int kSoftLimitMax = 340;
            public static final int kMaxOverrideDegreees = 15;
            public static final double kAllowedErrorDegrees = 0.75;
            public static final double kAllowedErrorRotations = kGearRatio*kAllowedErrorDegrees/360;
            public static final double kLengthInches = 35;
            public static final double kMassLb = 5.5;
            public static final double kMassDistribution = 0.686;
            public static final ArmSegment kSegment = new ArmSegment(kLengthInches, kMassLb, kMassDistribution, 0.5);

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
            //public static final float[] kSoftLimits = new float[] { (float) Units.degreesToRotations(kSoftLimitMax),(float) Units.degreesToRotations(kSoftLimitMin) };
            public static final float[] kSoftLimits = null;
            public static final int kContinuousCurrent = 20;
            public static final int kPeakCurrent = 60;

            public static final double kMaxRotatorRPM = 3000;
            public static final double kMinRotatorRPM = 0;

            public static final SparkMaxRotatingSubsystem.Config kConfig = new SparkMaxRotatingSubsystem.Config(
                    kMotorID, 0, kMotorType, kIdleMode,
                    kGearRatio, kP, kI, kD, kFF, kIz, kMaxOutput, kMinOutput,
                    kMaxRotatorRPM, kMinRotatorRPM, kMaxAcceleration, kAllowedErrorDegrees,
                    kSoftLimits, kContinuousCurrent, kPeakCurrent);
        }

        public static final class Wrist {

            // Wrist
            public static final double kAbsoluteEncoderVerticalOffset = RobotConstants.robotSpecificConstants
                    .get("kWristAbsoluteEncoderVerticalOffset");
            public static final double kGearRatio = (RobotConstants.robotSpecificConstants.get("kWristGearRatio"))
                    .intValue();
            public static final double kMaxOverrideDegrees = 90;
            public static final double kAllowedErrorDegrees = 1.5;
            public static final double kAllowedErrorRotations = kGearRatio* kAllowedErrorDegrees/360;
            public static final ArmSegment kWristSegment = new ArmSegment(Claw.kLengthInches, Claw.kMassLB,
                    Claw.kMassDistribution, 0);

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

            public static final SparkMaxRotatingSubsystem.Config kConfig = new SparkMaxRotatingSubsystem.Config(
                    kMotorID, 0, kMotorType, kIdleMode,
                    kGearRatio, kP, kI, kD, kFF, kIz, kMaxOutput, kMinOutput,
                    kMaxRotatorRPM, kMinRotatorRPM, kMaxAcceleration, kAllowedErrorRotations,
                    kSoftLimits, kContinuousCurrent, kPeakCurrent);

        }


        public static final class Claw {

            public static final int kMotorID = 6;
            public static final double kLengthInches = 4;
            public static final double kMassLB = 5;
            public static final double kMassDistribution = 1;
            public static final double kRollingSpeed = 1.0;
            public static final double kEjectingSpeed = -0.40;
            public static final double kCurrentMax = 30.0;
            public static final double kIdleSpeed = 0.05;
            public static final double kEjectTime = 0.6;
            public static final int kCurrentSpikeIterations = 10;
            ;
        }
    }

    public static final class Led {
        public static final int kPort = 0;
        public static final int kStartIndex = 0;
        public static final int kEndindex = 61;
    }
    public static final class CubeIntake{


        public static final int kMotorID = 9;
        

        public static final double kRollingSpeed = -0.5;
        // public static final double kEjectingSpeed = +0.8;
        public static final double kEjectingSpeed = +1.0;
        public static final double kCurrentMax = 30.0;
        public static final double kIdleSpeed = -0.08;
        public static final double kEjectTime = 1.6;
        public static final int kCurrentSpikeIterations = 11;
        
        public static final class Deployer{

        public static final int kSlot = 3;
                public static final int kAbsoluteEncoderID = 3;
                public static final Motor_Type kMotorType = MotorConfig.Motor_Type.NEO;
                public static final IdleMode kIdleMode = IdleMode.kCoast;
                public static final int kMotorID=16;
                public static final double kP = 0.008;
                public static final double kI = 0;
                public static final double kD = 0;
                public static final double kFF = 0.000176;
                public static final double kIz = 0;

                public static final double kMaxOutput = 1;
                public static final double kMinOutput = -1;
                public static final double kMaxAcceleration = 1900;
                public static final double kAllowedErrorDegrees = 0.35;

                public static final float[] kSoftLimits = null;
                public static final int kContinuousCurrent = 10;
                public static final int kPeakCurrent = 32;

                public static final double kGearRatio = RobotConstants.robotSpecificConstants.get("kDeployerGearRatio").intValue();
                public static final double kAllowedErrorRotations = kGearRatio* kAllowedErrorDegrees/360;
                public static final double kMaxRotatorRPM = 3500;
                public static final double kMinRotatorRPM = 0;
                // public static final double kMaxFlipOutRotations = -8.142;
                public static final double kTimeDown=0.2;
                public static final double kTimeUp=0.2;
                public static final double kMotorSpeed=-0.5;
                public static final int  kMaxAccelDownwards = 700;
                public static final int kMaxAccelUpwards = 1900;
                public static final double kAbsoluteEncoderVerticalOffset = RobotConstants.robotSpecificConstants.get("kDeployerAbsoluteEncoderVerticalOffset");
                // public static final double kAbsoluteEncoderPostionAtFlipoutZero = RobotConstants.robotSpecificConstants.get("kAbsoluteEncoderPostionAtFlipoutZero");


                public static final frc.team670.robot.subsystems.CubeIntake.Deployer.Config kConfig= new frc.team670.robot.subsystems.CubeIntake.Deployer.Config(
                kMotorID, kSlot, kMotorType, kIdleMode, kGearRatio, kContinuousCurrent, kPeakCurrent);
                public static final double kDeployedIdleSpeed = -0.05;
                


              
                }
} 

}
