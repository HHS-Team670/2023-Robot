package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.LEDSubsystem.LEDColor;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Claw extends MustangSubsystemBase {
    public enum Status {
        EJECTING, INTAKING, IDLE;
    }
    public enum GamePiece{
        NONE,CONE,CUBE;
    }

    protected Timer m_timer = new Timer();

    private SparkMAXLite motor;
    private Claw.Status status;
    private Claw.GamePiece gamepiece=Claw.GamePiece.CONE;

    private final String CLAW_MOTOR_CURRENT, CLAW_STATE, CLAW_IS_FULL;

    private int currentSpikeCounter = 0;
    private boolean isFull = true;
    private double ejectingSpeed =
            RobotConstants.Arm.Claw.kEjectingSpeed;
    private int heldCounter=0;

    private LED led;

    private static Claw mInstance;

    public static synchronized Claw getInstance() {
        mInstance = mInstance == null ? new Claw() : mInstance;
        return mInstance;
    }

    public Claw() {
        motor = SparkMAXFactory.buildSparkMAX(
                RobotConstants.Arm.Claw.kMotorID,
                SparkMAXFactory.defaultConfig, Motor_Type.NEO);
        status = Status.IDLE;
        this.led = LED.getInstance();

        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
        CLAW_MOTOR_CURRENT = getName() + "/MotorCurrent";
        CLAW_STATE = getName() + "/State";
        CLAW_IS_FULL = getName() + "/IsFull";
    }

    public LED getLed() {
        return led;
    }

    public void setLED(LED led) {
        this.led = led;
    }

    public boolean isFull() {
        return this.isFull;
    }

    public void startEjecting() {
        this.startEjecting(RobotConstants.Arm.Claw.kEjectingSpeed);
    }

    /**
     * Ejects the held item at the given speed
     * 
     * @param ejectingSpeed Should be <0. The more negative, the faster the claw will run backwards.
     */
    public void startEjecting(double ejectingSpeed) {
        this.ejectingSpeed = ejectingSpeed;
        if (this.status != Status.EJECTING) {
            this.isFull = true;
        }
        setStatus(Status.EJECTING);
        m_timer.restart();
    }

    public void startIntaking() {
        if (this.status != Status.INTAKING) {
            this.isFull = false;
        }
        setStatus(Status.INTAKING);
    }

    /**
     * Sets the claw to an IDLE state Please note that "IDLE" does not mean "stopped"!
     */
    public void setIdle() {
        setStatus(Status.IDLE);
    }
    public void setGamePiece(Claw.GamePiece gamepiece) {
        // if(!isFull()||gamepiece==GamePiece.NONE){
            this.gamepiece = gamepiece;
        // }
        
    }

    /**
     * Private method, only intended to be used by the public set() methods
     * 
     * @param status
     */
    private void setStatus(Claw.Status status) {
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
            if(gamepiece == GamePiece.CONE){
                motor.set(-RobotConstants.Arm.Claw.kIdleSpeed);//We may want different idle speeeds later on
            }else{
                motor.set(RobotConstants.Arm.Claw.kIdleSpeed);
            }
            if (DriverStation.isTeleopEnabled()) {
                if(!isFull()||heldCounter>20){
                    
                        // if(this.gamepiece==GamePiece.CONE){
                        //     led.solidhsv(LEDColor.YELLOW);
                        // }else{
                        //     led.solidhsv(LEDColor.PURPLE);
                        // }     
                }else if(isFull()){
                    heldCounter++;
                }
                    
            }

                break;
            case INTAKING:
            if(gamepiece == GamePiece.CONE){
                motor.set(-RobotConstants.Arm.Claw.kRollingSpeed);
            } else{
                motor.set(RobotConstants.Arm.Claw.kRollingSpeed);
            }
                
                if (motor.getOutputCurrent() > RobotConstants.Arm.Claw.kCurrentMax) {
                    currentSpikeCounter++;
                    if (currentSpikeCounter > RobotConstants.Arm.Claw.kCurrentSpikeIterations) {
                        isFull = true;
                        if (DriverStation.isTeleopEnabled()) {
                            // led.solidhsv(LEDColor.GREEN);
                        }
                       
                        // OI.getDriverController().rumble(0.5, 0.5);
                        // OI.getOperatorController().rumble(0.5, 0.5);
                        currentSpikeCounter = 0;
                        setIdle();
                    }
                } else {
                    currentSpikeCounter = 0;
                }
                break;

            case EJECTING:
                if(gamepiece == GamePiece.CONE){
                    motor.set(-ejectingSpeed);
                }else{
                    motor.set(ejectingSpeed);
                }

                if (m_timer.hasElapsed(RobotConstants.Arm.Claw.kEjectTime)) {
                    isFull = false;
                    m_timer.stop();
                    // if (DriverStation.isTeleopEnabled()) {
                    //     if(this.gamepiece==GamePiece.CONE){
                    //         led.solidhsv(LEDColor.YELLOW);
                    //     }else{
                    //         led.solidhsv(LEDColor.PURPLE);
                    //     }
                    // }
                }
                break;
            default:
                motor.set(0);
        }
    }

    @Override
    public void debugSubsystem() {
        Logger.getInstance().recordOutput(CLAW_MOTOR_CURRENT, motor.getOutputCurrent());
        Logger.getInstance().recordOutput(CLAW_STATE, status.toString());
        Logger.getInstance().recordOutput(CLAW_IS_FULL,isFull());
    }

}
