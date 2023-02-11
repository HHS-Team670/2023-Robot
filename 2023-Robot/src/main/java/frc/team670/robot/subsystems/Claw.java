package frc.team670.robot.subsystems;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
// SparkMAX is used for the motor control.
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotMap;

import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Claw extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, IDLE;
    }

    private static final double ROLLING_SPEED = 0.3;
    public static final double CURRENT_MAX = 25.0;
    private static final double IDLE_SPEED = 0.05;

    private SparkMAXLite leader, follower;

    private Claw.Status status;
    
    class MotorConfig {

    }

    public Claw() {
        List<SparkMAXLite> motorControllers = SparkMAXFactory.buildSparkMAXPair(RobotMap.LEFT_CLAW, 6, true, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
        leader = motorControllers.get(0);
        follower = motorControllers.get(1);
        status = Status.IDLE;
        leader.setIdleMode(IdleMode.kBrake);
        follower.setIdleMode(IdleMode.kBrake);
    }
    
    public void setStatus (Claw.Status status) {
        this.status = status;   
    }

    private void stopAll() {
        leader.set(0);
    }

    @Override
    // Checking for hardware breaks within the motors.
    public HealthState checkHealth() {
        if (leader == null || leader.isErrored() || follower == null || follower.isErrored()) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }
    
    
    @Override
    public void mustangPeriodic() {
        debugSubsystem();

        switch(status) {
            case IDLE:
                leader.set(IDLE_SPEED);
                break;
            case INTAKING:
                leader.set(ROLLING_SPEED);
                break;
            case EJECTING:
                leader.set(-ROLLING_SPEED);
                break;
            default:
                leader.set(0);
        }

        if(leader.getOutputCurrent() > CURRENT_MAX) {
            setStatus(Status.IDLE);
        }
    }

    @Override
    public void debugSubsystem() {
        // SmartDashboard.putNumber("leader current", leader.getOutputCurrent());
        // SmartDashboard.putNumber("follower current", leader.getOutputCurrent());
        SmartDashboard.putString("Claw state", status.toString());
    }

}