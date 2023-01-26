package frc.team670.robot.subsystems.arm;



import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import com.revrobotics.REVLibError;
public class Shoulder extends SparkMaxRotatingSubsystemLeaderFollower {



    //TODO: Fix Constants
    /*
     * PID and SmartMotion constants for the Shoulder joint
     */
    public static class Config extends SparkMaxRotatingSubsystemLeaderFollower.Config {

        	@Override
		public int getLeaderDeviceID() {
			// TODO Auto-generated method stub
			return 0;
		}

		@Override
		public int getFollowerDeviceID() {
			// TODO Auto-generated method stub
			return 0;
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
    public static final Config SHOULDER_CONFIG = new Config();
    public Shoulder() {
        super(SHOULDER_CONFIG);
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
        REVLibError leaderRotatorError = super.leaderRotator.getLastError();
        REVLibError followerRotatorError = super.followerRotator.getLastError();
		if ((leaderRotatorError != null && leaderRotatorError != leaderRotatorError.kOk) || (followerRotatorError != null && followerRotatorError != followerRotatorError.kOk)) {
			return HealthState.RED;
		}
        
		return HealthState.GREEN;
        
     
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Shoulder Speed:",super.leaderRotator.get());
        
    }

	@Override
	public double getCurrentAngleInDegrees() {
		 double rotations = super.getRotatorEncoder().getPosition()/super.getRotatorEncoder().getCountsPerRevolution();//
         
        // convert rotations to angle here
        //double oldrot = (angle / 360) * this.ROTATOR_GEAR_RATIO
        // + ((int) (getUnadjustedPosition() / this.ROTATOR_GEAR_RATIO)) * this.ROTATOR_GEAR_RATIO;//reverse engineer
        double angle = 360 * (( rotations - getUnadjustedPosition()) / this.ROTATOR_GEAR_RATIO);
        return angle;
	}
}
