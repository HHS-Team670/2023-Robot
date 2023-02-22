
package frc.team670.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
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
            return 0.0011;
        }

        public double getI() {
            return 0;
        }

        public double getD() {
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
            return 3500;
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
            return 80;
        }

        public double getRotatorGearRatio() {
            return RobotConstants.ELBOW_GEAR_RATIO;
        }

        public IdleMode setRotatorIdleMode() {
            return IdleMode.kBrake;
        }

        @Override
        public double getMaxRotatorRPM() {
            return 2400;
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
    //TODO: Move to mustang lib after testing
    public double getSetpoint() {
        return setpoint;
    }

    /**
     * PRIVATE method to set position from absolute.
     * Do not use directly. Instead, use resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        clearSetpoint();
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double relativePosition = ((-1
                * (absEncoderPosition - (RobotConstants.ELBOW_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5)) + 1)
                * RobotConstants.ELBOW_GEAR_RATIO) % RobotConstants.ELBOW_GEAR_RATIO;
        REVLibError error = rotator_encoder.setPosition(relativePosition);
        SmartDashboard.putNumber("Elbow absEncoder position when reset", absEncoderPosition);
        SmartDashboard.putNumber("Elbow relEncoder position when reset", relativePosition);
        SmartDashboard.putString("Elbow error", error.toString());
        calculatedRelativePosition = relativePosition;
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

        if(!hasSetAbsolutePosition || !relativePositionIsSet) {
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
        hasSetAbsolutePosition = false;
        counter = 0;
        relativePositionIsSet = false;
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Elbow Speed:", super.rotator.get());
        SmartDashboard.putNumber("Elbow forward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kForward));
        SmartDashboard.putNumber("Elbow backward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kReverse));
        SmartDashboard.putNumber("Elbow position (deg)", getCurrentAngleInDegrees());
        SmartDashboard.putNumber("Elbow position (rotations)", super.rotator_encoder.getPosition());
        SmartDashboard.putNumber("Elbow current", super.rotator.getOutputCurrent());
        SmartDashboard.putNumber("Elbow abs encoder position", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Elbow setpoint (rotations)", setpoint); 
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
            if (counter > 25) { // Once it's maintained a constant value for long enough...
                setEncoderPositionFromAbsolute();
                hasSetAbsolutePosition = true;
            }
        } else if (!relativePositionIsSet) {
            if (Math.abs(super.rotator_encoder.getPosition() - calculatedRelativePosition) < 0.01) {
                relativePositionIsSet = true;
            } else {
                super.rotator_encoder.setPosition(calculatedRelativePosition);
            }
        }
    }
}
