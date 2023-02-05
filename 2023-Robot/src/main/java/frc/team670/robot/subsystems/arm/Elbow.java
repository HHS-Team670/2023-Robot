
package frc.team670.robot.subsystems.arm;

import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the Elbow joint. Uses only one motor
 * @author Armaan Aditi Kedar Gabriel Alexander Justin
 */
public class Elbow extends SparkMaxRotatingSubsystem {

    DutyCycleEncoder absEncoder;

    /*
     * PID and SmartMotion constants for the Shoulder joint
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return RobotMap.ELBOW_MOTOR;
        }

        public int getSlot() {
            return 1;
        }

        public Motor_Type getMotorType() {
            return MotorConfig.Motor_Type.NEO;
        }

        public double getP() {
            return 0.0001; // Good enough for 2/17
        }

        public double getI() {
            return 0;
        }

        public double getD() {
            return 0;
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
            return 5000;
        }

        public double getAllowedError() {
            return 0.1;
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
            return 500;
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
        setEncoderPositionFromAbsolute();

    }

    public void setEncoderPositionFromAbsolute() {
        clearSetpoint();
        rotator_encoder.setPosition(
                (absEncoder.getAbsolutePosition() - (RobotConstants.ELBOW_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5))
                        * RobotConstants.ELBOW_GEAR_RATIO);
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

        return HealthState.GREEN;
    }
    
    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Elbow Speed:", super.rotator.get());
        SmartDashboard.putNumber("elbow forward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kForward));
        SmartDashboard.putNumber("elbow backward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kReverse));
        SmartDashboard.putNumber("Elbow position (deg)", getCurrentAngleInDegrees());
        SmartDashboard.putNumber("Elbow abs encoder position", absEncoder.getAbsolutePosition());
        SmartDashboard.putString("Elbow health", checkHealth().toString());
    }

    @Override
    public void mustangPeriodic() {

		setEncoderPositionFromAbsolute();
        
    }
}
