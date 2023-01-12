package frc.team670.robot.subsystems;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class Arm extends MustangSubsystemBase{
        
    //init motors
    SparkMAXLite arm2Motor;
    
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

        SparkMAXLite armMotor;
        //constructor that inits motors and stuff
        public ArmSegment() {
            armMotor = SparkMAXFactory.buildSparkMAX(RobotMap.ARM_ONE_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);

        }

        //store motor speed
        double motorSpeed;
        public void setSpeed(double speed){
            armMotor.set(speed);
        }
        //target
        //take a look at 2019 robot\
        

        public ArmState getStatus(){
            return armState;
        }
    }

}