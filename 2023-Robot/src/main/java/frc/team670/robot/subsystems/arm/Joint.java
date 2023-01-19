package frc.team670.robot.subsystems.arm;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
// import com.revrobotics.CANSparkMax.IdleMode;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;

public abstract class Joint extends SparkMaxRotatingSubsystem {
    

    //constructor that inits motors and stuff
    
    public Joint( Config config) {
        super(config);
    }

    @Override
    public boolean getTimeout() {
        // TODO Auto-generated method stub
        return false;
    }

    

    @Override
    public double getCurrentAngleInDegrees() {
        // TODO Auto-generated method stub
        //RelativeEncoder encoder = super.getRotatorEncoder();
        double rotations = 0.5;//
        // convert rotations to angle here
        //double oldrot = (angle / 360) * this.ROTATOR_GEAR_RATIO
        // + ((int) (getUnadjustedPosition() / this.ROTATOR_GEAR_RATIO)) * this.ROTATOR_GEAR_RATIO;//reverse engineer
        double angle = 360 * (( rotations - getUnadjustedPosition()) / this.ROTATOR_GEAR_RATIO);
        return angle;
    }

 



    @Override
    public void debugSubsystem() {
        
    }
}


