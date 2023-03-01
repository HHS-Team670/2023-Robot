package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.arm.Arm;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, IDLE;
    }

    private SparkMAXLite motor;
    private int currentSpikeCounter = 0;
    private int ejectCounter = 0;
    private Claw.Status status;
    private Arm arm;
    private boolean isFull = false;
    private double ejectingSpeed = RobotConstants.CLAW_EJECTING_SPEED;

    public Claw(Arm arm) {
        this.arm = arm;
        motor = SparkMAXFactory.buildSparkMAX(RobotMap.CLAW_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO);
        motor.setInverted(true);
        status = Status.IDLE;
        motor.setIdleMode(IdleMode.kBrake);
    }

    public Arm getArm() {
        return arm;
    }

    public boolean isFull() {
        return this.isFull;
    }

    public void startEjecting() {
        this.startEjecting(RobotConstants.CLAW_EJECTING_SPEED);
    }

    /**
     * Ejects the held item at the given speed
     * @param ejectingSpeed Should be <0. The more negative, the faster the claw will run backwards.
     */
    public void startEjecting(double ejectingSpeed) {
        this.ejectingSpeed = ejectingSpeed;
        setStatus(Status.EJECTING);
    }

    public void startIntaking() {
        setStatus(Status.INTAKING);
    }

    /**
     * Sets the claw to an IDLE state
     * Please note that "IDLE" does not mean "stopped"!
     */
    public void setIdle() {
        setStatus(Status.IDLE);
    }

    /**
     * Private method, only intended to be used by the public set() methods
     * @param status
     */
    private void setStatus (Claw.Status status) {
        this.status = status;
    }

    /**
     * Checking for hardware breaks with the motor
     */
    @Override
    public HealthState checkHealth() {
        if (motor == null || motor.isErrored()) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        // debugSubsystem();

        switch (status) {
            case IDLE:
                motor.set(RobotConstants.CLAW_IDLE_SPEED);
                break;
            case INTAKING:
                if(isFull) {
                    setStatus(Status.IDLE);
                }
                motor.set(RobotConstants.CLAW_ROLLING_SPEED);
                break;

            case EJECTING:
                motor.set(ejectingSpeed);
                ejectCounter++;
                if (ejectCounter > RobotConstants.CLAW_EJECT_ITERATIONS) {
                    ejectCounter = 0;
                    isFull = false;
                }
                break;
            default:
                motor.set(0);
        }

        // If the current has spiked for more than 1/10th of a second, then set the status to idle
        if(this.status == Status.INTAKING) {
            if(motor.getOutputCurrent() > RobotConstants.CLAW_CURRENT_MAX) {
                currentSpikeCounter++;
                if(currentSpikeCounter > RobotConstants.CLAW_CURRENT_SPIKE_ITERATIONS) {
                    isFull = true;
                    setIdle();
                }
            } else {
                currentSpikeCounter = 0;
            }

        }
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Claw motor current", motor.getOutputCurrent());
        SmartDashboard.putString("Claw state", status.toString());
    }

}