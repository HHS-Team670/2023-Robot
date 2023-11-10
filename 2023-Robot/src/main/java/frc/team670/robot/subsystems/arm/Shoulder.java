package frc.team670.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;

import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;

/**
 * Represents the shoulder joint. The shoulder uses a leader-follower SparkMax pair
 * 
 * @author Armaan, Kedar, Aditi, Justin, Alexander, Gabriel, Srinish, Sanatan
 */
public class Shoulder extends SparkMaxRotatingSubsystem {

    private DutyCycleEncoder absEncoder;
    private SparkMAXLite follower;
    private boolean hasSetAbsolutePosition = false;
    private int counter = 0;
    private double previousReading = 0.0;
    private double calculatedRelativePosition = 0.0;
    private boolean relativePositionIsSet = false;
    private double offset = 0;
    private double orgTargetAngle = 0;

    private final String SHOULDER_POSITION_DEG, SHOULDER_ABS_ENCODER_POS, SHOULDER_POSITION_ROT, SHOULDER_SETPOINT_ROT, SHOULDER_CURRENT, SHOULDER_ABS_ENCODER_POS_WHEN_RESET, SHOULDER_ERROR, SHOULDER_REL_ENCODER_POS_WHEN_RESET;
    public static final double SHOULDER_ARBITRARY_FF = 0.5;


    public Shoulder() {
        super(RobotConstants.Arm.Shoulder.kConfig);
        super.getRotator().setInverted(true);
        follower = SparkMAXFactory
                .setPermanentFollower(RobotConstants.Arm.Shoulder.kFollowerMotorID, mRotator, true);
        follower.setIdleMode(IdleMode.kBrake);
        absEncoder = new DutyCycleEncoder(RobotConstants.Arm.Shoulder.kAbsoluteEncoderID);

            SHOULDER_POSITION_DEG = getName() + "/position (deg)";
            SHOULDER_ABS_ENCODER_POS = getName() + "/abs encoder position";
            SHOULDER_POSITION_ROT = getName() + "/position (rotations)";
            SHOULDER_SETPOINT_ROT = getName() + "/setpoint (rotations)";
            SHOULDER_CURRENT = getName() + "/current";
            SHOULDER_ABS_ENCODER_POS_WHEN_RESET = getName() + "/absEncoder position when reset";
            SHOULDER_REL_ENCODER_POS_WHEN_RESET = getName() + "/relEncoder position when reset";
            SHOULDER_ERROR = getName() + "/error";
    }

    /**
     * Returns whether or not the relative position has been properly set from the absEncoder. When
     * resetPositionFromAbsolute() gets called, this will temporarily be false.
     */
    public boolean isRelativePositionSet() {
        return relativePositionIsSet;
    }

    @Override
    public boolean getTimeout() {
        return false;
    }

    @Override
    public boolean setSystemTargetAngleInDegrees(double targetAngle) {
        orgTargetAngle = targetAngle;
        return super.setSystemTargetAngleInDegrees(targetAngle + offset);
    }

    private void setOffset(double offset) {
        if (Math.abs(offset) > RobotConstants.Arm.Shoulder.kMaxOverrideDegrees) {
            this.offset = RobotConstants.Arm.Shoulder.kMaxOverrideDegrees * this.offset
                    / Math.abs(this.offset);
        } else {
            this.offset = offset;
        }
        setSystemTargetAngleInDegrees(orgTargetAngle);

    }

    public void resetOffset() {
        setOffset(0);

    }

    public void addOffset(double offset) {
        setOffset(this.offset + offset);
    }

    @Override
    public HealthState checkHealth() {
        REVLibError leaderRotatorError = super.mRotator.getLastError();
        REVLibError followerRotatorError = follower.getLastError();

        boolean leaderOK = (leaderRotatorError == REVLibError.kOk);
        boolean followerOK = (followerRotatorError == REVLibError.kOk);

        if (!leaderOK && !followerOK) {
            // Logger.consoleLog("Shoulder error! Leader error is " + leaderRotatorError.toString());
            // Logger.consoleLog(
            //         "Shoulder error! Follower error is " + followerRotatorError.toString());
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
        double relativePosition = super.mEncoder.getPosition();
        Logger.getInstance().recordOutput(SHOULDER_POSITION_DEG, getCurrentAngleInDegrees());
        Logger.getInstance().recordOutput(SHOULDER_POSITION_ROT, relativePosition);
        Logger.getInstance().recordOutput(SHOULDER_ABS_ENCODER_POS, absEncoder.getAbsolutePosition());
        Logger.getInstance().recordOutput(SHOULDER_SETPOINT_ROT, mSetpoint);
        Logger.getInstance().recordOutput(SHOULDER_CURRENT, super.getRotator().getOutputCurrent());
        sendAngleToDashboard();

    }

    /**
     * PRIVATE method to set position from absolute. DO NOT USE DIRECTLY. Instead, use
     * resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double previousPositionRot = super.mEncoder.getPosition();
        if (absEncoderPosition != 0.0) {

            double relativePosition = ((-1
                    * (absEncoderPosition
                            - (RobotConstants.Arm.Shoulder.kAbsoluteEncoderVerticalOffset - 0.5))
                    + 1) * RobotConstants.Arm.Shoulder.kGearRatio)
                    % RobotConstants.Arm.Shoulder.kGearRatio;

            if (calculatedRelativePosition == 0.0
                    || Math.abs(360 * ((previousPositionRot - relativePosition)
                            / kConfig.kRotatorGearRatio())) < 5.0) {
                clearSetpoint();
                REVLibError error = mEncoder.setPosition(relativePosition);
                Logger.getInstance().recordOutput(SHOULDER_ABS_ENCODER_POS_WHEN_RESET,
                        absEncoderPosition);
                Logger.getInstance().recordOutput(SHOULDER_REL_ENCODER_POS_WHEN_RESET,
                        relativePosition);
                Logger.getInstance().recordOutput(SHOULDER_ERROR, error.toString());
                calculatedRelativePosition = relativePosition;
            }

        }
    }

    @Override
    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(mEncoder.getPosition(), mSetpoint,
                RobotConstants.Arm.Shoulder.kAllowedErrorRotations));
    }

    @Override
    public void mustangPeriodic() {
        if (!hasSetAbsolutePosition) { // before it's set an absolute position...
            double position = absEncoder.getAbsolutePosition();
            if (Math.abs(previousReading - position) < 0.02 && position != 0.0) { // If the current
                                                                                  // reading is
                                                                                  // PRECISELY
                                                                                  // 0, then it's
                                                                                  // not valid.
                counter++; // increases the counter if the current reading is close enough to the
                           // last
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
            double position = super.mEncoder.getPosition();
            // Logger.consoleLog("Shoulder relative position = " + position
            //         + ", calculatedRelativePosition = " + calculatedRelativePosition);
            // Logger.consoleLog("Shoulder relativePositionIsSet = " + this.relativePositionIsSet);
            if (Math.abs(position - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.mEncoder.setPosition(calculatedRelativePosition);
            }
        }
    }

    public void sendAngleToDashboard() {
        Logger.getInstance().recordOutput(SHOULDER_POSITION_DEG, getCurrentAngleInDegrees());
    }
}
