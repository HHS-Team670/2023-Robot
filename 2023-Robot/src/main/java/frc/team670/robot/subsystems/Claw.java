package frc.team670.robot.subsystems;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
// SparkMAX is used for the motor control.
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Claw extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, OFF, IDLE;
    }

    private double leftSpeed, rightSpeed; 

    // public int overCount = 0;
    // public int overCountCap = 25;

    private SparkMAXLite left, right;
    private Claw.Status stat;
    
    // Placeholder
    
    public Claw() {
        left = SparkMAXFactory.buildFactorySparkMAX(RobotMap.LEFT_CLAW, Motor_Type.NEO_550);
        right = SparkMAXFactory.buildFactorySparkMAX(RobotMap.RIGHT_CLAW, Motor_Type.NEO_550);
        left.setFollow(right);
        
        leftSpeed = 0;
        rightSpeed = 0;
        // left.setIdleMode(IdleMode.kBrake);
        // right.setIdleMode(IdleMode.kBrake);
    }
    
    public void setStatus (Claw.Status stat) {
        this.stat = stat;   

        if (this.stat == Status.INTAKING) {
            this.intaking();
        } else if(this.stat == Status.EJECTING) {
            this.eject();
        } else if (this.stat == Status.IDLE) {
            this.idle();
        } else {
            this.stopAll();
        }
    }

    private void intaking() {
        leftSpeed = -RobotConstants.ROLLING_SPEED;
        rightSpeed = RobotConstants.ROLLING_SPEED;
    }

    private void eject() {
        rightSpeed = -RobotConstants.ROLLING_SPEED;
        leftSpeed = RobotConstants.ROLLING_SPEED;

        // overCount--;

        // if ((right.getOutputCurrent() < 1) && (left.getOutputCurrent() < 1) && overCount < -25) {
        //     setStatus(Status.IDLE);
        //     overCount = 0;
        // }
    }

    private void idle() {      
        leftSpeed = -RobotConstants.IDLE_SPEED;
        rightSpeed = RobotConstants.IDLE_SPEED;

        // if (left.getOutputCurrent() > CURRENT_MAX)
        // {
        //     overCount++;
        //     if(overCount > overCountCap)
        //     {
        //         leftSpeed = -IDLE_SPEED;
        //         rightSpeed = IDLE_SPEED;
        //     }
        // }
    }
    private void stopAll() {
        leftSpeed = 0;
        rightSpeed = 0;
        //overCount = 0;
    }

    // Checks if a game object is currently being held.
    // public boolean isFull() {
    //     return overCount > 0;
    // }

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
        left.set(leftSpeed);
        right.set(rightSpeed);
        debugSubsystem();
        // if(this.stat != Status.EJECTING)
        // {
        //     stopClaw();
        // }
    }

    @Override
    public void debugSubsystem() 
    {
        SmartDashboard.putNumber("left current", left.getOutputCurrent());
        SmartDashboard.putNumber("right current", right.getOutputCurrent());
    }

    public double getLeftCurrent() {
        return left.getOutputCurrent();
    }

    public double getRightCurrent() {
        return right.getOutputCurrent();
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