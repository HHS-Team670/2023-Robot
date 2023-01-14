package frc.team670.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class Arm extends MustangSubsystemBase{
        
    //init motors
    SparkMaxLite armMotor;
    SparkMAXLite armMotor2;
    
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
        SparkMAXLite armMotor2;
        //constructor that inits motors and stuff
        public ArmSegment() {
            armMotor2 = SparkMAXFactory.buildSparkMAX(RobotMap.ARM_TWO_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO_Brushless);
        }

        //store motor speed
        double motorSpeed;
        public void setSpeed(double speed){
            armMotor.set(speed);
            armMotor2.set(speed);
        }
        //target
        //take a look at 2019 robot\
        

        public ArmState getStatus(){
            return armState;
        }
    }

}