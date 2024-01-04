
package frc.team670.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.ConsoleLogger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.robot.constants.RobotConstants;

/**
 * Represents the wrist joint. Uses only one motor
 * 
 * @author Armaan Aditi Kedar Gabriel Alexander Justin
 */
public class Wrist extends SparkMaxRotatingSubsystem {

    private DutyCycleEncoder absEncoder;
    private boolean hasSetAbsolutePosition = false;
    private int counter = 0;
    private double previousReading = 0.0;
    private double calculatedRelativePosition = 0.0;
    private boolean relativePositionIsSet = false;
    private double errorCounter = 0;
    private double offset = 0;
    private double orgTargetAngle;
    private final String WRIST_POSITION_DEG_KEY, WRIST_SETPOINT_ROT_KEY, WRIST_ABSENCODER_POS_RESET_KEY, WRIST_RELENCODER_POS_RESET_KEY, WRIST_CURRENT_KEY, WRIST_ERROR_KEY;


    public Wrist() {
        super(RobotConstants.Arm.Wrist.kConfig);
        absEncoder = new DutyCycleEncoder(RobotConstants.Arm.Wrist.kAbsoluteEncoderID);
        super.getRotator().setInverted(false);
        WRIST_POSITION_DEG_KEY = getName() + "/CurrentAngleDeg";
        WRIST_SETPOINT_ROT_KEY = getName() + "/SetpointRot";
        WRIST_ABSENCODER_POS_RESET_KEY = getName() + "/AbsEncoderPosWhenReset";
        WRIST_RELENCODER_POS_RESET_KEY = getName() + "/RelEncoderPosWhenReset";
        WRIST_ERROR_KEY = getName() + "/Error";
        WRIST_CURRENT_KEY = getName() + "/Current";
    }


    /**
     * PRIVATE method to set position from absolute. DO NOT USE DIRECTLY. Instead, use
     * resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double previousPositionRot = super.mEncoder.getPosition();

        if (absEncoderPosition != 0.0) {
            double relativePosition = ((1 * (absEncoderPosition
                    - (RobotConstants.Arm.Wrist.kAbsoluteEncoderVerticalOffset - 0.5)) + 1)
                    * RobotConstants.Arm.Wrist.kGearRatio) % RobotConstants.Arm.Wrist.kGearRatio;

            if (calculatedRelativePosition == 0.0
                    || Math.abs(360 * ((previousPositionRot - relativePosition)
                            / kConfig.kRotatorGearRatio())) < 20.0) {
                clearSetpoint();

                REVLibError error = mEncoder.setPosition(relativePosition);
                Logger.getInstance().recordOutput(WRIST_ABSENCODER_POS_RESET_KEY,
                        absEncoderPosition);
                        Logger.getInstance().recordOutput(WRIST_RELENCODER_POS_RESET_KEY, relativePosition);
                        Logger.getInstance().recordOutput(WRIST_ERROR_KEY, error.toString());
                calculatedRelativePosition = relativePosition;
            }

        }
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
        this.offset = offset;
        setSystemTargetAngleInDegrees(orgTargetAngle+offset);

    }
    public void resetOffset() {
        setOffset(0);

    }

    public void addOffset(double offset) {
        setOffset(this.offset + offset);
    }
    

    @Override
    public HealthState checkHealth() {
        REVLibError rotatorError = super.mRotator.getLastError();

        if (rotatorError != null && rotatorError != REVLibError.kOk) {
            ConsoleLogger.consoleLog("Wrist error! Rotator Error is " + rotatorError.toString());
            errorCounter++;
        } else {
            errorCounter = 0;
        }

        if (errorCounter >= 20) {
            return HealthState.RED;
        }


        if (!hasSetAbsolutePosition || !relativePositionIsSet) {
            return HealthState.YELLOW;
        }

        return HealthState.GREEN;
    }

    /**
     * Returns whether or not the relative position has been properly set from the absEncoder. When
     * resetPositionFromAbsolute() gets called, this will temporarily be false.
     */
    public boolean isRelativePositionSet() {
        return relativePositionIsSet;
    }

    /**
     * PUBLIC method to reset the wrist position from the absolute encoder.
     */
    public void resetPositionFromAbsolute() {
        setEncoderPositionFromAbsolute();
    }

    @Override
    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(mEncoder.getPosition(), mSetpoint,
                RobotConstants.Arm.Wrist.kAllowedErrorRotations));
    }

    @Override
    public void debugSubsystem() {
        double relativePosition = super.mEncoder.getPosition();

        Logger.getInstance().recordOutput(WRIST_POSITION_DEG_KEY, getCurrentAngleInDegrees());
        Logger.getInstance().recordOutput(WRIST_RELENCODER_POS_RESET_KEY, relativePosition);
        Logger.getInstance().recordOutput(WRIST_ABSENCODER_POS_RESET_KEY, absEncoder.getAbsolutePosition());
        Logger.getInstance().recordOutput(WRIST_SETPOINT_ROT_KEY, mSetpoint);
        Logger.getInstance().recordOutput(WRIST_CURRENT_KEY, super.getRotator().getOutputCurrent());
        sendAngleToDashboard();

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
            if (counter > 25) { // Once it's maintained a constant value for long enough...
                setEncoderPositionFromAbsolute();
                hasSetAbsolutePosition = true;
            }
        } else if (!relativePositionIsSet) {
            if (Math.abs(super.mEncoder.getPosition() - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.mEncoder.setPosition(calculatedRelativePosition);
            }
        }
    }

    public void sendAngleToDashboard() {
        Logger.getInstance().recordOutput(WRIST_POSITION_DEG_KEY, getCurrentAngleInDegrees());
    }
}
