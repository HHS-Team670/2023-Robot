
package frc.team670.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;

import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.robot.constants.RobotConstants;

/**
 * Represents the Elbow joint. Uses only one motor
 * 
 * @author Armaan Aditi Kedar Gabriel Alexander Justin Sanatan Srinish
 */
public class Elbow extends SparkMaxRotatingSubsystem {

    private DutyCycleEncoder absEncoder;
    private boolean hasSetAbsolutePosition = false;
    private int counter = 0;
    private double previousReading = 0.0;
    private double calculatedRelativePosition = 0.0;
    private boolean relativePositionIsSet = false;
    private double offset = 0;
    private double orgTargetAngle = 0;
    private double errorCounter = 0;

    private final String ELBOW_POSITION_DEG, ELBOW_ABS_ENCODER_POS, ELBOW_POSITION_ROT, ELBOW_SETPOINT_ROT, ELBOW_CURRENT, ELBOW_ABS_ENCODER_POS_WHEN_RESET, ELBOW_REL_ENCODER_POS_WHEN_RESET, ELBOW_ERROR;

    // constructor that inits motors and stuff

    public Elbow() {
        super(RobotConstants.Arm.Elbow.kConfig);
        absEncoder = new DutyCycleEncoder(RobotConstants.Arm.Elbow.kAbsoluteEncoderID);
        super.getRotator().setInverted(true);
        ELBOW_POSITION_DEG = getName() + "/position (deg)";
        ELBOW_ABS_ENCODER_POS = getName() + "/abs encoder position";
        ELBOW_POSITION_ROT = getName() + "/position (rotations)";
        ELBOW_SETPOINT_ROT = getName() + "/setpoint (rotations)";
        ELBOW_CURRENT = getName() + "/current";
        ELBOW_ABS_ENCODER_POS_WHEN_RESET = getName() + "/absEncoder position when reset";
        ELBOW_REL_ENCODER_POS_WHEN_RESET = getName() + "relEncoder position when reset";
        ELBOW_ERROR = getName() + "/error";
    }

    /**
     * PRIVATE method to set position from absolute. Do not use directly. Instead, use
     * resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double previousPositionRot = super.mEncoder.getPosition();

        if (absEncoderPosition != 0.0) {
            double relativePosition = ((-1 * (absEncoderPosition
                    - (RobotConstants.Arm.Elbow.kAbsoluteEncoderVerticalOffset - 0.5)) + 2)
                    * RobotConstants.Arm.Elbow.kGearRatio) % RobotConstants.Arm.Elbow.kGearRatio;

            if (calculatedRelativePosition == 0.0
                    || Math.abs(360 * ((previousPositionRot - relativePosition)
                            / kConfig.kRotatorGearRatio())) < 10.0) {
                clearSetpoint();
                REVLibError error = mEncoder.setPosition(relativePosition);
                Logger.getInstance().recordOutput(ELBOW_ABS_ENCODER_POS_WHEN_RESET,
                        absEncoderPosition);
                Logger.getInstance().recordOutput(ELBOW_REL_ENCODER_POS_WHEN_RESET, relativePosition);
                Logger.getInstance().recordOutput(ELBOW_ERROR, error.toString());
                calculatedRelativePosition = relativePosition;
            }
        }

    }

    @Override
    public boolean getTimeout() {
        return false;
    }

    @Override
    public HealthState checkHealth() {
        REVLibError rotatorError = super.mRotator.getLastError();

        if (rotatorError != null && rotatorError != REVLibError.kOk) {
            // Logger.consoleLog("Elbow error! Rotator error is " + rotatorError.toString());
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
     * Public method to reset the position from the absolute position.
     */
    public void resetPositionFromAbsolute() {
        setEncoderPositionFromAbsolute();
    }

    // @Override
    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(mEncoder.getPosition(), mSetpoint,
        RobotConstants.Arm.Elbow.kAllowedErrorRotations));
    }

    @Override
    public boolean setSystemTargetAngleInDegrees(double targetAngle) {
        orgTargetAngle = targetAngle;
        return super.setSystemTargetAngleInDegrees(targetAngle+offset);
    }

    private void setOffset(double offset) {
        if (Math.abs(offset) > RobotConstants.Arm.Elbow.kMaxOverrideDegreees) {
            this.offset = RobotConstants.Arm.Elbow.kMaxOverrideDegreees * this.offset
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
    public void debugSubsystem() {
        double relativePosition = super.mEncoder.getPosition();
        Logger.getInstance().recordOutput(ELBOW_POSITION_DEG,getCurrentAngleInDegrees());
        
        Logger.getInstance().recordOutput(ELBOW_POSITION_ROT, relativePosition);
        Logger.getInstance().recordOutput(ELBOW_ABS_ENCODER_POS, absEncoder.getAbsolutePosition());
        Logger.getInstance().recordOutput(ELBOW_SETPOINT_ROT, mSetpoint);
        Logger.getInstance().recordOutput(ELBOW_CURRENT, super.getRotator().getOutputCurrent());
        sendAngleToDashboard();
        // Logger.getInstance().recordOutput("Elbow motor power: ", super.mRotator.get());
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
            // Logger.consoleLog("Elbow/relative position = " + position
            //         + ", calculatedRelativePosition = " + calculatedRelativePosition);
            if (Math.abs(position - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.mEncoder.setPosition(calculatedRelativePosition);
            }
            // Logger.consoleLog("Elbow relativePositionIsSet = " + this.relativePositionIsSet);
        }
    }

    public void sendAngleToDashboard() {
        Logger.getInstance().recordOutput(positionDeg, getCurrentAngleInDegrees());
    }
}
