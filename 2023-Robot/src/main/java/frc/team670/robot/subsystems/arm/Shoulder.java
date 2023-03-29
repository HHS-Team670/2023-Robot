package frc.team670.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import com.revrobotics.REVLibError;

/**
 * Represents the shoulder joint. The shoulder uses a leader-follower SparkMax
 * pair
 * 
 * @author Armaan, Kedar, Aditi, Justin, Alexander, Gabriel, Srinish, Sanatan
 */
public class Shoulder extends SparkMaxRotatingSubsystem {

    private DutyCycleEncoder absEncoder;
    private SparkMAXLite follower;
    private boolean hasSetAbsolutePosition = false;
    int counter = 0;
    double previousReading = 0.0;
    double calculatedRelativePosition = 0.0;
    boolean relativePositionIsSet = false;
    private double offset = 0;

    private final String positionDeg = "Shoulder position (deg)";
    private final String absEncoderPos = "Shoulder abs encoder position";
    private final String positionRot = "Shoulder position (rotations)";
    private final String setpointRot = "Shoulder setpoint (rotations)";

    /*
     * PID and SmartMotion constants for the Shoulder joint
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.SHOULDER_LEADER_MOTOR;
        }

        public int getSlot() {
            return 0;
        }

        public Motor_Type getMotorType() {
            return MotorConfig.Motor_Type.NEO;
        }

        public double getP() {
            return 0.0005;
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0.00005;
        }

        public double getFF() {
            return 0.00017618;
        }

        public double getIz() {
            return 0;
        }

        public double getMaxOutput() {
            return 1;
        }

        public double getMinOutput() {
            return -1;
        }

        public double getMaxAcceleration() {
            return 2500;
        }

        public double getAllowedError() {
            return RobotConstants.SHOULDER_GEAR_RATIO * 0.2 / 360;
        }

        public boolean enableSoftLimits() {
            return true;
        }

        public float[] getSoftLimits() {
            return new float[] { convertDegreesToRotations(RobotConstants.SHOULDER_SOFT_LIMIT_MAX),
                    convertDegreesToRotations(RobotConstants.SHOULDER_SOFT_LIMIT_MIN) };
        }

        public int getContinuousCurrent() {
            return 20;
        }

        public int getPeakCurrent() {
            return 60;
        }

        public double getRotatorGearRatio() {
            return RobotConstants.SHOULDER_GEAR_RATIO;
        }

        public IdleMode setRotatorIdleMode() {
            return IdleMode.kBrake;
        }

        @Override
        public double getMaxRotatorRPM() {
            return 1500;
        }

        @Override
        public double getMinRotatorRPM() {
            return 0;
        }
    }

    // constructor that inits motors and stuff
    public static final Config SHOULDER_CONFIG = new Config();

    public Shoulder() {
        super(SHOULDER_CONFIG);
        super.getRotator().setInverted(true);
        follower = SparkMAXFactory.setPermanentFollower(RobotMap.SHOULDER_FOLLOWER_MOTOR, rotator, true);
        follower.setIdleMode(IdleMode.kBrake);
        absEncoder = new DutyCycleEncoder(RobotMap.SHOULDER_ABSOLUTE_ENCODER);
    }

    /**
     * Returns whether or not the relative position has been properly set from the absEncoder.
     * When resetPositionFromAbsolute() gets called, this will temporarily be false.
     */
    public boolean isRelativePositionSet() {
        return relativePositionIsSet;
    }

    /**
     * Updates the arbitraryFF value to counteract gravity
     * @param voltage The calculated voltage, returned from VoltageCalculator
     */
    public void updateArbitraryFeedForward(double voltage) {
        if (setpoint != SparkMaxRotatingSubsystem.NO_SETPOINT) {
            rotator_controller.setReference(setpoint,
                    SparkMAXLite.ControlType.kSmartMotion, super.SMARTMOTION_SLOT,
                    voltage);
        }
    }

    @Override
    public boolean getTimeout() {
        return false;
    }

    public void setOffset(double offset) {
        if (Math.abs(offset) > RobotConstants.SHOULDER_MAX_OVERRIDE_DEGREES) {
            this.offset = RobotConstants.SHOULDER_MAX_OVERRIDE_DEGREES * this.offset / Math.abs(this.offset);
        } else {
            this.offset = offset;
        }

    }

    public double getOffset() {
        return offset;
    }

    @Override
    public HealthState checkHealth() {
        REVLibError leaderRotatorError = super.rotator.getLastError();
        REVLibError followerRotatorError = follower.getLastError();

        boolean leaderOK = (leaderRotatorError == REVLibError.kOk);
        boolean followerOK = (followerRotatorError == REVLibError.kOk);

        if (!leaderOK && !followerOK) {
            Logger.consoleError("Shoulder error! Leader error is " + leaderRotatorError.toString());
            Logger.consoleError("Shoulder error! Follower error is " + followerRotatorError.toString());
            return HealthState.RED;
        }

        if ((leaderOK && !followerOK) || (!leaderOK && followerOK) || !hasSetAbsolutePosition
                || !relativePositionIsSet) {
            return HealthState.YELLOW;
        }

        return HealthState.GREEN;

    }

    /**
     * Public method to reset arm's relative position from absolute
     */
    public void resetPositionFromAbsolute() {
        setEncoderPositionFromAbsolute();
    }

    @Override
    public void debugSubsystem() {
        double relativePosition = super.rotator_encoder.getPosition();
        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
        SmartDashboard.putNumber(positionRot, relativePosition);
        SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(setpointRot, setpoint);

    }

    /**
     * PRIVATE method to set position from absolute.
     * DO NOT USE DIRECTLY. Instead, use resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double previousPositionRot = super.rotator_encoder.getPosition();
        if(absEncoderPosition != 0.0) {

            double relativePosition = ((-1
                    * (absEncoderPosition - (RobotConstants.SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5)) + 1)
                    * RobotConstants.SHOULDER_GEAR_RATIO) % RobotConstants.SHOULDER_GEAR_RATIO;
            
            if(calculatedRelativePosition == 0.0 || Math.abs(360 * ((previousPositionRot - relativePosition) / this.ROTATOR_GEAR_RATIO)) < 3.0) {
                clearSetpoint();
                REVLibError error = rotator_encoder.setPosition(relativePosition);
                SmartDashboard.putNumber("Shoulder absEncoder position when reset", absEncoderPosition);
                SmartDashboard.putNumber("Shoulder relEncoder position when reset", relativePosition);
                SmartDashboard.putString("Shoulder error", error.toString());
                calculatedRelativePosition = relativePosition;
            }

        }
    }

    // TODO: Move to mustang lib after testing;
    public double getSetpoint() {
        return setpoint;
    }

    @Override
    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(rotator_encoder.getPosition(), setpoint, RobotConstants.SHOULDER_ALLOWED_ERR_DEG));
    }

    @Override
    public void mustangPeriodic() {
        if (!hasSetAbsolutePosition) { // before it's set an absolute position...
            double position = absEncoder.getAbsolutePosition();
            if (Math.abs(previousReading - position) < 0.02 && position != 0.0) { // If the current reading is PRECISELY
                                                                                  // 0, then it's not valid.
                counter++; // increases the counter if the current reading is close enough to the last
                           // reading.
                           // We do this because when the absEncoder gets initialized, its reading
                           // fluctuates drastically at the start.
            } else {
                counter = 0;
                previousReading = position;
            }
            if (counter > 100) { // Once it's maintained a constant value for long enough...
                setEncoderPositionFromAbsolute();
                hasSetAbsolutePosition = true;
            }
        } else if (!relativePositionIsSet) {
            double position = super.rotator_encoder.getPosition();
            Logger.consoleLog("Shoulder relative position = " + position + ", calculatedRelativePosition = "
                    + calculatedRelativePosition);
            Logger.consoleLog("Shoulder relativePositionIsSet = " + this.relativePositionIsSet);
            if (Math.abs(position - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.rotator_encoder.setPosition(calculatedRelativePosition);
            }
        }
    }

    public void sendAngleToDashboard() {
        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
    }
}
