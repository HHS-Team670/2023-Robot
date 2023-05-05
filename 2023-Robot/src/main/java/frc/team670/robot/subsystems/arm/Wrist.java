
package frc.team670.robot.subsystems.arm;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
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

    private final String positionDeg = "Wrist position (deg)";
    private final String absEncoderPos = "Wrist abs encoder position";
    private final String positionRot = "Wrist position (rotations)";
    private final String setpointRot = "Wrist setpoint (rotations)";


    public Wrist() {
        super(RobotConstants.Arm.Wrist.kConfig);
        absEncoder = new DutyCycleEncoder(
                RobotConstants.Arm.Wrist.kAbsoluteEncoderID);
        super.getmRotator().setInverted(false);


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
                    - (RobotConstants.Arm.Wrist.kAbsoluteEncoderVerticalOffset
                            - 0.5))
                    + 1) * RobotConstants.Arm.Wrist.kGearRatio)
                    % RobotConstants.Arm.Wrist.kGearRatio;

            if (calculatedRelativePosition == 0.0
                    || Math.abs(360 * ((previousPositionRot - relativePosition)
                            / kConfig.kRotatorGearRatio())) < 20.0) {
                clearSetpoint();

                REVLibError error = mEncoder.setPosition(relativePosition);
                SmartDashboard.putNumber("Wrist absEncoder position when reset",
                        absEncoderPosition);
                SmartDashboard.putNumber("Wrist relEncoder position when reset", relativePosition);
                SmartDashboard.putString("Wrist error", error.toString());
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
            Logger.consoleError("Wrist error! Rotator Error is " + rotatorError.toString());
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
                RobotConstants.Arm.Wrist.kAllowedErrorDegrees));
    }

    @Override
    public void debugSubsystem() {
        double relativePosition = super.mEncoder.getPosition();

        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
        SmartDashboard.putNumber(positionRot, relativePosition);
        SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(setpointRot, mSetpoint);

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
        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
    }
}
