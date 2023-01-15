 package frc.team670.robot.subsystems.arm;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.SparkMaxLite;

public class Arm extends MustangSubsystemBase{
        
    //init motors
    SparkMaxLite shoulderMotor;
    SparkMAXLite elbowMotor;
    ArmSegment shoulder;
    ArmSegment elbow;
    
    // public enum ArmState{
    //     LEVEL1,
    //     LEVEL2,
    //     LEVEL1BACK,
    //     LEVEL2BACK,
    //     SHELF,
    //     SHELFBACK
    // }

    //private static ArmState armState = ArmState.LEVEL1;

    
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