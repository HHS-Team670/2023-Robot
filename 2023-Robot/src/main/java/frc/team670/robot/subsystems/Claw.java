package frc.team670.robot.subsystems;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.LEDSubsystem.LEDColor;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, IDLE;
    }
    public enum GamePiece{
        NONE,CONE,CUBE;
    }

    private SparkMAXLite motor;
    private Claw.Status status;
    private Claw.GamePiece gamepiece=Claw.GamePiece.NONE;

    private final String currentKey = "Claw motor current";
    private final String clawStateKey = "Claw state";

    private int currentSpikeCounter = 0;
    private int ejectCounter = 0;
    private boolean isFull = false;
    private double ejectingSpeed =
            RobotConstants.Arm.Claw.kEjectingSpeed;
    

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
                            led.solidhsv(LEDColor.GREEN);
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
                
                ejectCounter++;
                if (ejectCounter > RobotConstants.Arm.Claw.kEjectIterations) {
                    ejectCounter = 0;
                    isFull = false;
                    if (DriverStation.isTeleopEnabled()) {
                        if(this.gamepiece==GamePiece.CONE){
                            led.solidhsv(LEDColor.YELLOW);
                        }else{
                            led.solidhsv(LEDColor.PURPLE);
                        }
                    }
                }
                break;
            default:
                motor.set(0);
        }
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber(currentKey, motor.getOutputCurrent());
        SmartDashboard.putString(clawStateKey, status.toString());
    }

}
