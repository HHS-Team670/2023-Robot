package frc.team670.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
 * Represents the shoulder joint. The shoulder uses a leader-follower SparkMax pair
 * @author Armaan, Kedar, Aditi, Justin, Alexander, Gabriel
 */
public class Shoulder extends SparkMaxRotatingSubsystem {

    DutyCycleEncoder absEncoder;
    private SparkMAXLite follower;
    private boolean hasSetAbsolutePosition = false;

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
            return 0.0001;
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
            return 1000;
        }

        public double getAllowedError() {
            return RobotConstants.SHOULDER_GEAR_RATIO * 2.0/360;
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
            return 960;
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
        follower = SparkMAXFactory.setPermanentFollower(RobotMap.SHOULDER_FOLLOWER_MOTOR, rotator);
        follower.setInverted(true);
        absEncoder = new DutyCycleEncoder(RobotMap.SHOULDER_ABSOLUTE_ENCODER);
        //setEncoderPositionFromAbsolute();

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

        // if(!hasSetAbsolutePosition) {
        //     return HealthState.YELLOW;
        // }

        if(!leaderOK && !followerOK) {
            return HealthState.RED;
        }

        if((leaderOK && !followerOK) || (!leaderOK && followerOK)) {
            return HealthState.YELLOW;
        }

        return HealthState.GREEN;

    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Shoulder Speed:", super.rotator.get());
        SmartDashboard.putNumber("Shoulder forward soft limit",
                super.rotator.getSoftLimit(SoftLimitDirection.kForward));
        SmartDashboard.putNumber("Shoudler backward soft limit",
                super.rotator.getSoftLimit(SoftLimitDirection.kReverse));
        SmartDashboard.putNumber("Shoulder position (deg)", getCurrentAngleInDegrees());
        SmartDashboard.putNumber("Shoulder abs encoder position", absEncoder.getAbsolutePosition());

        SmartDashboard.putString("Shoulder health", checkHealth().toString());

    }

    /**
     * Sets the rotator encoder's reference position to the constant obtained from
     * the absolute encoder corresponding to that position.
     */
    public void setEncoderPositionFromAbsolute() {
        clearSetpoint();
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        rotator_encoder.setPosition(
                (absEncoderPosition - (RobotConstants.SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5))
                        * RobotConstants.SHOULDER_GEAR_RATIO);
        SmartDashboard.putNumber("shoulder position at init", absEncoderPosition);
        SmartDashboard.putNumber("shoulder rotator encoder setPosition", absEncoderPosition - (RobotConstants.SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5)
        * RobotConstants.SHOULDER_GEAR_RATIO);
    }

	@Override
	public void mustangPeriodic() {
        if(!hasSetAbsolutePosition) {
            if(absEncoder.getAbsolutePosition() != 0.000) { //If it's PRECISELY 0, then it doesn't have a valid position yet
            	setEncoderPositionFromAbsolute();
                hasSetAbsolutePosition = true;
            }
        }
		
	}
}
