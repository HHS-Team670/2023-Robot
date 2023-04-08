
package frc.team670.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
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

    private final String positionDeg = "Elbow position (deg)";
    private final String absEncoderPos = "Elbow abs encoder position";
    private final String positionRot = "Elbow position (rotations)";
    private final String setpointRot = "Elbow setpoint (rotations)";
    private final String current = "Elbow current";

    // constructor that inits motors and stuff

    public Elbow() {
        super(RobotConstants.Arm.Elbow.kConfig);
        absEncoder = new DutyCycleEncoder(
                frc.team670.robot.constants.RobotConstants.Arm.Elbow.kAbsoluteEncoderID);
        super.getRotator().setInverted(true);
    }

    /**
     * Updates the arbitraryFF value to counteract gravity
     * 
     * @param voltage The calculated voltage, returned from VoltageCalculator
     */
    public void updateArbitraryFeedForward(double voltage) {
        if (setpoint != SparkMaxRotatingSubsystem.NO_SETPOINT) {
            rotator_controller.setReference(setpoint, SparkMAXLite.ControlType.kSmartMotion,
                    super.SMARTMOTION_SLOT, voltage);
        }
    }

    // TODO: Move to mustang lib after testing
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * PRIVATE method to set position from absolute. Do not use directly. Instead, use
     * resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double previousPositionRot = super.rotator_encoder.getPosition();

        if (absEncoderPosition != 0.0) {
            double relativePosition = ((-1 * (absEncoderPosition
                    - (frc.team670.robot.constants.RobotConstants.Arm.Elbow.kAbsoluteEncoderVerticalOffsetRadians
                            - 0.5))
                    + 1) * frc.team670.robot.constants.RobotConstants.Arm.Elbow.kGearRatio)
                    % frc.team670.robot.constants.RobotConstants.Arm.Elbow.kGearRatio;

            if (calculatedRelativePosition == 0.0
                    || Math.abs(360 * ((previousPositionRot - relativePosition)
                            / this.ROTATOR_GEAR_RATIO)) < 10.0) {
                clearSetpoint();
                REVLibError error = rotator_encoder.setPosition(relativePosition);
                SmartDashboard.putNumber("Elbow absEncoder position when reset",
                        absEncoderPosition);
                SmartDashboard.putNumber("Elbow relEncoder position when reset", relativePosition);
                SmartDashboard.putString("Elbow error", error.toString());
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
        REVLibError rotatorError = super.rotator.getLastError();

        if (rotatorError != null && rotatorError != REVLibError.kOk) {
            Logger.consoleError("Elbow error! Rotator error is " + rotatorError.toString());
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
    // public boolean hasReachedTargetPosition() {
    //     return (MathUtils.doublesEqual(rotator_encoder.getPosition(), setpoint,
    //             frc.team670.robot.constants.RobotConstants.Arm.Elbow.kAllowedErrorDegrees));
    // }

    @Override
    public void setSystemTargetAngleInDegrees(double targetAngle) {
        orgTargetAngle = targetAngle;
        super.setSystemTargetAngleInDegrees(orgTargetAngle + offset);
    }

    private void setOffset(double offset) {
        if (Math.abs(
                offset) > frc.team670.robot.constants.RobotConstants.Arm.Elbow.kMaxOverrideDegreees) {
            this.offset = frc.team670.robot.constants.RobotConstants.Arm.Elbow.kMaxOverrideDegreees
                    * this.offset / Math.abs(this.offset);
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
        double relativePosition = super.rotator_encoder.getPosition();

        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
        SmartDashboard.putNumber(positionRot, relativePosition);
        SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(setpointRot, setpoint);
        SmartDashboard.putNumber(current, super.getRotator().getOutputCurrent());
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
            double position = super.rotator_encoder.getPosition();
            Logger.consoleLog("Elbow relative position = " + position
                    + ", calculatedRelativePosition = " + calculatedRelativePosition);
            if (Math.abs(position - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.rotator_encoder.setPosition(calculatedRelativePosition);
            }
            Logger.consoleLog("Elbow relativePositionIsSet = " + this.relativePositionIsSet);
        }
    }

    public void sendAngleToDashboard() {
        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
    }
}
