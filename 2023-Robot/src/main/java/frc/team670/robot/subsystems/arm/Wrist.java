
package frc.team670.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

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
                frc.team670.robot.constants.RobotConstants.Arm.Wrist.kAbsoluteEncoderID);
        super.getRotator().setInverted(false);


    }

    /**
     * Calculated voltage using VoltageCalculator
     * 
     * @param voltage
     */
    public void updateArbitraryFeedForward(double voltage) {
        if (setpoint != SparkMaxRotatingSubsystem.NO_SETPOINT) {
            rotator_controller.setReference(setpoint, SparkMAXLite.ControlType.kSmartMotion,
                    super.SMARTMOTION_SLOT, voltage);
        }
    }

    /**
     * PRIVATE method to set position from absolute. DO NOT USE DIRECTLY. Instead, use
     * resetPositionFromAbsolute()
     */
    public void setEncoderPositionFromAbsolute() {
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double previousPositionRot = super.rotator_encoder.getPosition();

        if (absEncoderPosition != 0.0) {
            double relativePosition = ((1 * (absEncoderPosition
                    - (frc.team670.robot.constants.RobotConstants.Arm.Wrist.kAbsoluteEncoderVerticalOffsetRadians
                            - 0.5))
                    + 1) * frc.team670.robot.constants.RobotConstants.Arm.Wrist.kGearRatio)
                    % frc.team670.robot.constants.RobotConstants.Arm.Wrist.kGearRatio;

            if (calculatedRelativePosition == 0.0
                    || Math.abs(360 * ((previousPositionRot - relativePosition)
                            / this.ROTATOR_GEAR_RATIO)) < 20.0) {
                clearSetpoint();

                REVLibError error = rotator_encoder.setPosition(relativePosition);
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
        REVLibError rotatorError = super.rotator.getLastError();

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
        return (MathUtils.doublesEqual(rotator_encoder.getPosition(), setpoint,
                frc.team670.robot.constants.RobotConstants.Arm.Wrist.kAllowedErrorDegrees));
    }

    @Override
    public void debugSubsystem() {
        double relativePosition = super.rotator_encoder.getPosition();

        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
        SmartDashboard.putNumber(positionRot, relativePosition);
        SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(setpointRot, setpoint);

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
            if (Math.abs(super.rotator_encoder.getPosition() - calculatedRelativePosition) < 0.5) {
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
