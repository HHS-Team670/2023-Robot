package frc.team670.robot.subsystems.arm;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
// import com.revrobotics.CANSparkMax.IdleMode;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;


public class Elbow extends Joint {
    



    //TODO: Fix Constants
    /*
     * PID and SmartMotion constants for the Shoulder joint
     */
    public static class Config extends SparkMaxRotatingSubsystem.Config {

        public int getDeviceID() {
            return 0; //RobotMap.FLIP_OUT;
        }

        public int getSlot() {
            return 0;
        }

        public Motor_Type getMotorType() {
            return null;//MotorConfig.Motor_Type.NEO;
        }

        public double getP() {
            return 0.00015; // Good enough for 2/17
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
            return 0.35;
        }

        public boolean enableSoftLimits() {
            return false;
        }

        public float[] setSoftLimits() {
            return null;
        }

        public int getContinuousCurrent() {
            return 20;
        }

        public int getPeakCurrent() {
            return 80;
        }

        public double getRotatorGearRatio() {
            return 0;//FLIPOUT_GEAR_RATIO;
        }

        public IdleMode setRotatorIdleMode() {
            return null;//IdleMode.kCoast;
        }

        @Override
        public double getMaxRotatorRPM() {
            return 3500;
        }

        @Override
        public double getMinRotatorRPM() {
            return 0;
        }

    }

    //constructor that inits motors and stuff
    public static final Config ELBOW_CONFIG = new Config();
    public Elbow() {
        super(ELBOW_CONFIG);
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
        REVLibError rotatorError = super.rotator.getLastError();
		if (rotatorError != null && rotatorError != rotatorError.kOk) {
			return HealthState.RED;
		}
        
		return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        
        
    }

    @Override
    public void debugSubsystem() {
        return manymanymanymanymanymanymanymanybeans;

    }
}
