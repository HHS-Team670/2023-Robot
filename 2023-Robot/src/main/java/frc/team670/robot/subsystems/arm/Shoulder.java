package frc.team670.robot.subsystems.arm;



import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

import org.ejml.simple.ConvertToDenseException;

import com.revrobotics.REVLibError;
public class Shoulder extends SparkMaxRotatingSubsystem {
    
    DutyCycleEncoder absEncoder;
    private SparkMAXLite follower;


    //TODO: Fix Constants
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
            return 1900;
        }

        public double getAllowedError() {
            return 0.07;
        }

        public boolean enableSoftLimits() {
            return true;
        }

        public float[] setSoftLimits() {
            return new float[]{convertDegreesToRotations(RobotConstants.SHOULDER_SOFT_LIMIT_MAX), convertDegreesToRotations(RobotConstants.SHOULDER_SOFT_LIMIT_MIN)};
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
            return 3500;
        }

        @Override
        public double getMinRotatorRPM() {
            return 0;
        }

        public float convertDegreesToRotations(float d) {
        return (float) ((d / 360) * RobotConstants.SHOULDER_GEAR_RATIO);
        }
    }

    //constructor that inits motors and stuff
    public static final Config SHOULDER_CONFIG = new Config();
    public Shoulder() {
        super(SHOULDER_CONFIG);

        rotator_encoder.setPosition(0);
        
        follower = SparkMAXFactory.setPermanentFollower(RobotMap.SHOULDER_FOLLOWER_MOTOR, rotator);
        
        absEncoder = new DutyCycleEncoder(RobotMap.SHOULDER_ABSOLUTE_ENCODER);
        setEncoderPositionFromAbsolute();

    }
    
    @Override
    public boolean getTimeout() {
        return false;
    }

    @Override
    public void moveByPercentOutput(double output) {

    }

    

    @Override
    public HealthState checkHealth() {
        REVLibError leaderRotatorError = super.rotator.getLastError();
        //REVLibError followerRotatorError = follower.getLastError();
		if ((leaderRotatorError != null && leaderRotatorError != leaderRotatorError.kOk) /*|| (followerRotatorError != null && followerRotatorError != followerRotatorError.kOk)*/) {
			return HealthState.RED;
		}
        
		return HealthState.GREEN;
        
     
    }

    @Override
    public void mustangPeriodic() {
        SmartDashboard.putNumber("shoulder forward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kForward));
        SmartDashboard.putNumber("shoulder backward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kReverse));
        //setSystemTargetAngleInDegrees(SmartDashboard.getNumber("shoulderTarget", 0));
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Shoulder Speed:",super.rotator.get());
        //SmartDashboard.putNumber("Shoulder position", super.rotator_encoder.getPosition());
        SmartDashboard.putNumber("Shoulder position (deg)", getCurrentAngleInDegrees());
        
    }

	@Override
	public double getCurrentAngleInDegrees() {
         double rotations = super.getRotatorEncoder().getPosition();
         
        // convert rotations to angle here
        //double oldrot = (angle / 360) * this.ROTATOR_GEAR_RATIO
        // + ((int) (getUnadjustedPosition() / this.ROTATOR_GEAR_RATIO)) * this.ROTATOR_GEAR_RATIO;//reverse engineer
        double angle = 360 * (( rotations ) / this.ROTATOR_GEAR_RATIO);
        return angle;
	}

    /**
     * Sets the rotator encoder's reference position to the constant obtained from
     * the absolute encoder corresponding to that position.
     */
    public void setEncoderPositionFromAbsolute() {
        clearSetpoint();
        rotator_encoder.setPosition(
                (getAbsoluteEncoderRotations() - (RobotConstants.SHOULDER_ABSOLUTE_ENCODER_AT_VERTICAL - 0.5))
                        * RobotConstants.SHOULDER_GEAR_RATIO);
        // Logger.consoleLog("Encoder position set: %s", rotator_encoder.getPosition());
    }

    public double getAbsoluteEncoderRotations() {
        double pos = absEncoder.get();
        return pos;
    }
}
