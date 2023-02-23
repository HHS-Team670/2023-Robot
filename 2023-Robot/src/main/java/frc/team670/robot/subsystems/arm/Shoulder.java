package frc.team670.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.team670.mustanglib.utils.Logger;
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
    String relativePositionLog = "";

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
            return 80;
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

    @Override
    public HealthState checkHealth() {
        REVLibError leaderRotatorError = super.rotator.getLastError();
        REVLibError followerRotatorError = follower.getLastError();

        boolean leaderOK = (leaderRotatorError == REVLibError.kOk);
        boolean followerOK = (followerRotatorError == REVLibError.kOk);

        if (!leaderOK && !followerOK) {
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
        hasSetAbsolutePosition = false;
        counter = 0;
        relativePositionIsSet = false;
    }

    @Override
    public void debugSubsystem() {
        double relativePosition = super.rotator_encoder.getPosition();
        SmartDashboard.putNumber("Shoulder Speed:", super.rotator.get());
        SmartDashboard.putNumber("Shoulder forward soft limit",
                super.rotator.getSoftLimit(SoftLimitDirection.kForward));
        SmartDashboard.putNumber("Shoulder backward soft limit",
                super.rotator.getSoftLimit(SoftLimitDirection.kReverse));
        SmartDashboard.putNumber("Shoulder position (deg)", getCurrentAngleInDegrees());
        SmartDashboard.putNumber("Shoulder position (rotations)", relativePosition);
        SmartDashboard.putNumber("Shoulder abs encoder position", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Shoulder current", super.rotator.getOutputCurrent());
        SmartDashboard.putNumber("Shoulder setpoint (rotations)", setpoint);

        relativePositionLog += ("" + relativePosition + ", ");

    }

    /**
     * PRIVATE method to set position from absolute.
     * DO NOT USE DIRECTLY. Instead, use resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        clearSetpoint();
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double relativePosition = ((-1
                * (absEncoderPosition - (RobotConstants.SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5)) + 1)
                * RobotConstants.SHOULDER_GEAR_RATIO) % RobotConstants.SHOULDER_GEAR_RATIO;
        REVLibError error = rotator_encoder.setPosition(relativePosition);
        SmartDashboard.putNumber("Shoulder absEncoder position when reset", absEncoderPosition);
        SmartDashboard.putNumber("Shoulder relEncoder position when reset", relativePosition);
        SmartDashboard.putString("Shoulder error", error.toString());
        calculatedRelativePosition = relativePosition;
    }

    // TODO: Move to mustang lib after testing;
    public double getSetpoint() {
        return setpoint;
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
            Logger.consoleLog("Shoulder relative position = " + position + ", calculatedRelativePosition = " + calculatedRelativePosition);
            Logger.consoleLog("Shoulder relativePositionIsSet = " + this.relativePositionIsSet);
            if (Math.abs(position - calculatedRelativePosition) < 0.01) {
                relativePositionIsSet = true;
                Logger.consoleLog(relativePositionLog);
            } else {
                super.rotator_encoder.setPosition(calculatedRelativePosition);
            }
        }
    }
}
