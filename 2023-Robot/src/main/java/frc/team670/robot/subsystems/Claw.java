package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.LEDColor;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, IDLE;
    }

    private SparkMAXLite motor;
    private Claw.Status status;

    private int currentSpikeCounter = 0;
    private int ejectCounter = 0;
    private boolean isFull = false;
    private double ejectingSpeed = RobotConstants.CLAW_EJECTING_SPEED;

    private LED led;

    public Claw(LED led) {
        motor = SparkMAXFactory.buildSparkMAX(RobotMap.CLAW_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO);
        status = Status.IDLE;
        this.led = led;

        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
        
    }

    public LED getLed() {
        return led;
    }
    
    public void setLED(LED led){
        this.led = led;
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
        if(this.status != Status.EJECTING) {
            this.isFull = true;
        }
        setStatus(Status.EJECTING);
    }

    public void startIntaking() {
        if(this.status != Status.INTAKING) {
            this.isFull = false;
        }
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

        switch (status) {
            case IDLE:
                motor.set(RobotConstants.CLAW_IDLE_SPEED);
                break;
            case INTAKING:
                motor.set(RobotConstants.CLAW_ROLLING_SPEED);
                if(motor.getOutputCurrent() > RobotConstants.CLAW_CURRENT_MAX) {
                    currentSpikeCounter++;
                    if(currentSpikeCounter > RobotConstants.CLAW_CURRENT_SPIKE_ITERATIONS) {
                        isFull = true;
                        if(DriverStation.isTeleopEnabled()){
                            led.solidhsv(LEDColor.GREEN);
                    }
                        setStatus(Status.IDLE);
                        OI.getDriverController().rumble(0.5, 0.5);
                        OI.getOperatorController().rumble(0.5, 0.5);
                        currentSpikeCounter = 0;
                        setIdle();
                    }
                } else {
                    currentSpikeCounter = 0;
                }
                break;

            case EJECTING:
                motor.set(ejectingSpeed);
                ejectCounter++;
                if (ejectCounter > RobotConstants.CLAW_EJECT_ITERATIONS) {
                    ejectCounter = 0;
                    isFull = false;
                    if(DriverStation.isTeleopEnabled()){
                        led.off();
                }
                }
                break;
            default:
                motor.set(0);
        }
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Claw motor current", motor.getOutputCurrent());
        SmartDashboard.putString("Claw state", status.toString());
    }

}