package frc.team670.robot.subsystems;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
// SparkMAX is used for the motor control.
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.dataCollection.sensors.BeamBreak;
import frc.team670.mustanglib.subsystems.*;
import frc.team670.robot.constants.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Claw extends MustangSubsystemBase 
{

    public enum Status 
    {
        EJECTING, INTAKING, OFF;
    }

    private static final double ROLLING_SPEED = 0.3;
    private static final double CURRENT_MAX = 25.0;
    public int overCount = 0;
    public int overCountCap = 25;

    private SparkMAXLite left, right;
    private Claw.Status stat;
    
    // Placeholder
    
    public Claw() 
    {
        left = SparkMAXFactory.buildFactorySparkMAX(RobotMap.LEFT_CLAW, Motor_Type.NEO_550);
        right = SparkMAXFactory.buildFactorySparkMAX(RobotMap.RIGHT_CLAW, Motor_Type.NEO_550);
    
        // left.setIdleMode(IdleMode.kBrake);
        // right.setIdleMode(IdleMode.kBrake);
    }
    
    public void setStatus (Claw.Status stat)
    {
        this.stat = stat;   

        if(this.stat == Status.INTAKING)
        {
            this.intaking();
        }else if(this.stat == Status.EJECTING)
        {
            this.ejecting();
        }else 
        {
            this.stopAll();
        }
    }

    public void intaking()
    {
        stat = Status.INTAKING;

        left.set(-ROLLING_SPEED);
        right.set(ROLLING_SPEED);
    }

    public void ejecting()
    {
        stat = Status.EJECTING;

        right.set(-ROLLING_SPEED);
        left.set(ROLLING_SPEED);
        overCount=0;
    }

    public void stopAll() 
    {
        left.set(0);
        right.set(0);
    }

    // Checks if a game object is currently being held.
    public boolean isFull() 
    {
        return overCount > 0;
    }

    public void stopClaw()
    {
        if(left.getOutputCurrent() > CURRENT_MAX)
        {
            overCount++;
            if(overCount > overCountCap)
            {
                setStatus(Status.OFF);
            }
        }
    }

    @Override
    // Checking for hardware breaks within the motors.
    public HealthState checkHealth() {
        if (left == null || left.isErrored() || right == null || right.isErrored()) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }
    
    
    @Override
    // What is mustangPeriodic() used for? Is it used for updating the motors?
    public void mustangPeriodic() {
        debugSubsystem();
        
        //stopClaw();
        // if (isLeftJammed() || isRightJammed()) {
        //     left.set(-left.get());
        //     right.set(-right.get());
        // }

        stopClaw();
    }

    @Override
    public void debugSubsystem() 
    {
        SmartDashboard.putNumber("left current", left.getOutputCurrent());
        SmartDashboard.putNumber("right current", right.getOutputCurrent());
        
    }

    // Method(s) to detect if the claw is outtaking or intaking to further check what direction the motors should spin.
    // Perhaps there should be a sensor.

    // public void setSpinOfMotors() {
    //     // Still unsure of the motors spinning.
    //     // TODO: change directions based on Mech
    //     switch(stat) {
    //         case INTAKING:
    //             // Make the motors spins outwards.
    //             left.set(ROLLING_SPEED);
    //             right.set(-ROLLING_SPEED);
    //             break;
    //         case OUTTAKING:
    //             // Make the motors spin inwards.
    //             right.set(-ROLLING_SPEED);
    //             left.set(ROLLING_SPEED);
    //             break;
    //         case OFF:
    //             // If the motors are turned off.
    //             stopAll();
    //             break;   
    //     }
    // }

    // /**
    //  * Returns true if the left claw intake is jammed.
    //  * @return
    //  */
    // public boolean isLeftJammed() {
    //     double intakeCurrent = left.getOutputCurrent();
    //     if (intakeCurrent > 0.2) {
    //         if (intakeCurrent >= INTAKE_PEAK_CURRENT) {
    //             exceededCurrentLimitCountLeft++;
    //         } else {
    //             exceededCurrentLimitCountLeft = 0;
    //         }
    //         if (exceededCurrentLimitCountLeft >= 4) { // 4 consecutive readings higher than peak
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    // public boolean isRightJammed() {
    //     double intakeCurrent = right.getOutputCurrent();
    //     if (intakeCurrent > 0.2) {
    //         if (intakeCurrent >= INTAKE_PEAK_CURRENT) {
    //             exceededCurrentLimitCountRight++;
    //         } else {
    //             exceededCurrentLimitCountRight = 0;
    //         }
    //         if (exceededCurrentLimitCountRight >= 4) { // 4 consecutive readings higher than peak
    //             return true;
    //         }
    //     }
    //     return false;
    // }
    
}