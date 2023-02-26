
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

    /*
     * PID and SmartMotion constants for the Wrist joint
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.WRIST_MOTOR;
        }

        public int getSlot() {
            return 0;
        }

        public Motor_Type getMotorType() {
            return MotorConfig.Motor_Type.NEO_550;
        }

        public double getP() {
            return 0.00011; 
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
            return 15000;
        }

        public double getAllowedError() {
            return RobotConstants.WRIST_GEAR_RATIO * 0.2 / 360;
        }

        public boolean enableSoftLimits() {
            return false;
        }

        public float[] getSoftLimits() {
            return null;
        }

        public int getContinuousCurrent() {
            return 20;
        }

        public int getPeakCurrent() {
            return 20;
        }

        public double getRotatorGearRatio() {
            return RobotConstants.WRIST_GEAR_RATIO;
        }

        public IdleMode setRotatorIdleMode() {
            return IdleMode.kBrake;
        }

        @Override
        public double getMaxRotatorRPM() {
            return 6000;
        }

        @Override
        public double getMinRotatorRPM() {
            return 0;
        }

    }

    public static final Config WRIST_CONFIG = new Config();

    public Wrist() {
        super(WRIST_CONFIG);
        absEncoder = new DutyCycleEncoder(RobotMap.WRIST_ABSOLUTE_ENCODER);
        super.getRotator().setInverted(false);
        SmartDashboard.putNumber("wrist arbitary feed forward value", RobotConstants.WRIST_ARBITRARY_FF);


    }

    /**
     * Calculated voltage using VoltageCalculator
     * @param voltage
     */
    public void updateArbitraryFeedForward(double voltage) {
        if(setpoint != SparkMaxRotatingSubsystem.NO_SETPOINT){
            rotator_controller.setReference(setpoint,
                    SparkMAXLite.ControlType.kSmartMotion, super.SMARTMOTION_SLOT,
                    voltage);
        }
    }

    /**
     * PRIVATE method to set position from absolute.
     * DO NOT USE DIRECTLY. Instead, use resetPositionFromAbsolute()
     */
    public void setEncoderPositionFromAbsolute() {
        clearSetpoint();
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double relativePosition = ((1
                * (absEncoderPosition - (RobotConstants.WRIST_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5)) + 1)
                * RobotConstants.WRIST_GEAR_RATIO) % RobotConstants.WRIST_GEAR_RATIO;
        REVLibError error = rotator_encoder.setPosition(relativePosition);
        SmartDashboard.putNumber("Wrist absEncoder position when reset", absEncoderPosition);
        SmartDashboard.putNumber("Wrist relEncoder position when reset", relativePosition);
        SmartDashboard.putString("Wrist error", error.toString());
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
     * PUBLIC method to reset the wrist position from the absolute encoder.
     */
    public void resetPositionFromAbsolute() {
        hasSetAbsolutePosition = false;
        counter = 0;
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Wrist Speed:", super.rotator.get());
        SmartDashboard.putNumber("Wrist forward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kForward));
        SmartDashboard.putNumber("Wrist backward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kReverse));
        SmartDashboard.putNumber("Wrist position (deg)", getCurrentAngleInDegrees());
        SmartDashboard.putNumber("Wrist position (rotations)", super.rotator_encoder.getPosition());
        SmartDashboard.putNumber("Wrist current", super.rotator.getOutputCurrent());
        SmartDashboard.putNumber("Wrist abs encoder position", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Wrist setpoint (rotations)", setpoint);

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
            if (Math.abs(super.rotator_encoder.getPosition() - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.rotator_encoder.setPosition(calculatedRelativePosition);
            }
        }
    }
}
