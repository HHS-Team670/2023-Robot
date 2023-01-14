package frc.team670.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

public class Arm extends MustangSubsystemBase{
        
    //init motors
    SparkMaxLite armMotor1;
    SparkMAXLite armMotor2;
    ArmSegment arm1;
    ArmSegment arm2;
    
    public enum ArmState{
        LEVEL1,
        LEVEL2,
        LEVEL1BACK,
        LEVEL2BACK,
        SHELF,
        SHELFBACK
    }

    private static ArmState armState = ArmState.LEVEL1;

    static class ArmSegment{

        SparkMAXLite armMotor1;
        SparkMAXLite armMotor2;
        //constructor that inits motors and stuff
        public ArmSegment() {
            armMotor1 = SparkMAXFactory.buildSparkMAX(RobotMap.ARM_ONE_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO_Brushless);
            armMotor2 = SparkMAXFactory.buildSparkMAX(RobotMap.ARM_TWO_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO_Brushless);   
        }

        //store motor speed
        double motorSpeed;
        public void setSpeed(double speed){
            armMotor1.set(speed);
            armMotor2.set(speed);
        }
        //target
        //take a look at 2019 robot\
        

        public ArmState getStatus(){
            return armState;
        }
    }
    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub
    }

    @Override
    public void debugSubsystem() {
        // TODO Auto-generated method stub
        //SmartDashboard.putNumber("Balls", balls);
    }

}