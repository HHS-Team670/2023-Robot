
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
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

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

    private final String positionDeg = "Elbow position (deg)";
    private final String absEncoderPos = "Elbow abs encoder position";
    private final String positionRot = "Elbow position (rotations)";
    private final String setpointRot = "Elbow setpoint (rotations)";
    private final String current = "Elbow current";

    /*
     * PID and SmartMotion constants for the Elbow joint
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.ELBOW_MOTOR;
        }

        public int getSlot() {
            return 0;
        }

        public Motor_Type getMotorType() {
            return MotorConfig.Motor_Type.NEO;
        }

        public double getP() {
            // return 0.0011; //Value when chain popped on 3/1 and 3/2
            return 0.0007;
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            //return 0.00015;
            return 0.00015;
        }

        public double getFF() {
            return 0.000176;
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
            return 4000;
        }

        public double getAllowedError() {
            return RobotConstants.ELBOW_GEAR_RATIO * 0.2 / 360;
        }

        public boolean enableSoftLimits() {
            return true;
        }

        public float[] getSoftLimits() {
            return new float[] { convertDegreesToRotations(RobotConstants.ELBOW_SOFT_LIMIT_MAX),
                    convertDegreesToRotations(RobotConstants.ELBOW_SOFT_LIMIT_MIN) };
        }

        public int getContinuousCurrent() {
            return 20;
        }

        public int getPeakCurrent() {
            return 60;
        }

        public double getRotatorGearRatio() {
            return RobotConstants.ELBOW_GEAR_RATIO;
        }

        public IdleMode setRotatorIdleMode() {
            return IdleMode.kBrake;
        }

        @Override
        public double getMaxRotatorRPM() {
            return 3000;
        }

        @Override
        public double getMinRotatorRPM() {
            return 0;
        }

    }

    // constructor that inits motors and stuff
    public static final Config ELBOW_CONFIG = new Config();

    public Elbow() {
        super(ELBOW_CONFIG);
        absEncoder = new DutyCycleEncoder(RobotMap.ELBOW_ABSOLUTE_ENCODER);
        super.getRotator().setInverted(true);
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

    // TODO: Move to mustang lib after testing
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * PRIVATE method to set position from absolute.
     * Do not use directly. Instead, use resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        if(absEncoderPosition != 0.0) {
            clearSetpoint();
            double relativePosition = ((-1
                    * (absEncoderPosition - (RobotConstants.ELBOW_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5)) + 1)
                    * RobotConstants.ELBOW_GEAR_RATIO) % RobotConstants.ELBOW_GEAR_RATIO;
            REVLibError error = rotator_encoder.setPosition(relativePosition);
            SmartDashboard.putNumber("Elbow absEncoder position when reset", absEncoderPosition);
            SmartDashboard.putNumber("Elbow relEncoder position when reset", relativePosition);
            SmartDashboard.putString("Elbow error", error.toString());
            calculatedRelativePosition = relativePosition;
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
            return HealthState.RED;
        }

        if (!hasSetAbsolutePosition || !relativePositionIsSet) {
            return HealthState.YELLOW;
        }

        return HealthState.GREEN;
    }

    /**
     * Returns whether or not the relative position has been properly set from the absEncoder.
     * When resetPositionFromAbsolute() gets called, this will temporarily be false.
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

    @Override
    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(rotator_encoder.getPosition(), setpoint, RobotConstants.ELBOW_ALLOWED_ERR_DEG));
    }

    public void setOffset(double offset) {
        if (Math.abs(offset) > RobotConstants.ELBOW_MAX_OVERRIDE_DEGREES) {
            this.offset = RobotConstants.ELBOW_MAX_OVERRIDE_DEGREES * this.offset / Math.abs(this.offset);
        } else {
            this.offset = offset;
        }

    }

    public double getOffset() {
        return offset;
    }

    @Override
    public void debugSubsystem() {
        double relativePosition = super.rotator_encoder.getPosition();

        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
        SmartDashboard.putNumber(positionRot, relativePosition);
        SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(setpointRot, setpoint);
        SmartDashboard.putNumber(current,  super.getRotator().getOutputCurrent());
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
            Logger.consoleLog("Elbow relative position = " + position + ", calculatedRelativePosition = "
                    + calculatedRelativePosition);
            if (Math.abs(position - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.rotator_encoder.setPosition(calculatedRelativePosition);
            }
            Logger.consoleLog("Elbow relativePositionIsSet = " + this.relativePositionIsSet);
        }
    }
}
