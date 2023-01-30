
package frc.team670.robot.subsystems.arm;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.IdleMode;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;

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
import frc.team670.robot.constants.RobotMap;

import org.ejml.simple.ConvertToDenseException;

public class Elbow extends SparkMaxRotatingSubsystem {
    



    //TODO: Fix Constants
    /*
     * PID and SmartMotion constants for the Shoulder joint
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {
        public static final double ELBOW_GEAR_RATIO = 25.0;

        public int getDeviceID() {
            return 24; //RobotMap.FLIP_OUT;
        }

        public int getSlot() {
            return 0;
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

        public double getFF() { // Good enough for 2/17
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
            return 0.1;
        }

        public boolean enableSoftLimits() {
            return true;
        }

        public float[] setSoftLimits() {
            return new float[]{convertDegreesToRotations(340), convertDegreesToRotations(20)};
        }

        public int getContinuousCurrent() {
            return 20;
        }

        public int getPeakCurrent() {
            return 80;
        }

        public double getRotatorGearRatio() {
            return ELBOW_GEAR_RATIO;//FLIPOUT_GEAR_RATIO;
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
            return (float) ((d / 360) * ELBOW_GEAR_RATIO);
            }

    }

    //constructor that inits motors and stuff
    public static final Config ELBOW_CONFIG = new Config();
    public Elbow() {
        super(ELBOW_CONFIG);
        rotator_encoder.setPosition(12.5);

    }

    


    @Override
    public boolean getTimeout() {
        return false;
    }

    
    @Override
    public double getCurrentAngleInDegrees(){
        //double rotations = super.getRotatorEncoder().getPosition()/super.getRotatorEncoder().getCountsPerRevolution();//
        double rotations = super.getRotatorEncoder().getPosition();
         
        // convert rotations to angle here
        //double oldrot = (angle / 360) * this.ROTATOR_GEAR_RATIO
        // + ((int) (getUnadjustedPosition() / this.ROTATOR_GEAR_RATIO)) * this.ROTATOR_GEAR_RATIO;//reverse engineer
        double angle = 360 * (( rotations ) / this.ROTATOR_GEAR_RATIO);
        return angle;
    }
    

    @Override
    public HealthState checkHealth() {
        REVLibError rotatorError = super.rotator.getLastError();
		if (rotatorError != null && rotatorError != rotatorError.kOk) {
			return HealthState.RED;
		}
        
		return HealthState.GREEN;
    }


    @Override
    public void mustangPeriodic() {
        SmartDashboard.putNumber("elbow forward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kForward));
        SmartDashboard.putNumber("elbow backward soft limit", super.rotator.getSoftLimit(SoftLimitDirection.kReverse));
        
    }

    @Override
    public void debugSubsystem() {
        //return manymanymanymanymanymanymanymanybeans;
        SmartDashboard.putNumber("Elbow Speed:",super.rotator.get());
        SmartDashboard.putNumber("Elbow position (deg)", getCurrentAngleInDegrees());

    }



    /** 
        Sets the motor speed to output 
        output=[-1,1]
     */
	@Override
	public void moveByPercentOutput(double output) {
        super.rotator.set(output);
		
	}
}
