package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.robot.constants.RobotConstants;


public class Claw extends MustangSubsystemBase {

    public enum Status{
        EJECTING, INTAKING, IDLE;
    }

    public enum GamePiece{
        NONE, CUBE, CONE;
    }

    private SparkMAXLite motor;
    private Claw.Status status;
    private Claw.GamePiece gamepiece = GamePiece.CONE;

    private double ejectingSpeed = RobotConstants.Arm.Claw.kEjectingSpeed;
    private LED led;
    private boolean isFull = true;
    private Timer mTimer;

    private static Claw mInstance;

    public static synchronized Claw getInstance(){
        mInstance = mInstance == null ? new Claw() : mInstance;
        return mInstance;
    }

    public Claw(){
        motor = SparkMAXFactory.buildSparkMAX(RobotConstants.Arm.Claw.kMotorID, SparkMAXFactory.defaultConfig, Motor_Type.NEO);
        status = Status.IDLE;
        this.led = LED.getInstance();
    }

    public LED getLED(){
        return led;
    }

    public void setLED(LED led){
        this.led = led;
    }

    public boolean isFull(){
        return isFull;
    }

    public void setGamePiece(Claw.GamePiece gamepiece){
        this.gamepiece = gamepiece;
    }

    public void setStatus(Claw.Status status){
        this.status = status;
    }

    public void setIdle(){
        setStatus(Status.IDLE);
    }

    public void startEjecting(){
        this.startEjecting(RobotConstants.Arm.Claw.kEjectingSpeed);
    }

    public void startEjecting(double ejectingSpeed){
        this.ejectingSpeed = ejectingSpeed;
        if(this.status != Status.EJECTING){
            this.isFull = true;
        }
        setStatus(Status.EJECTING);
    }

    public void startIntaking(){
        if(this.status != Status.INTAKING){
            this.isFull = false;
        }
        setStatus(Status.INTAKING);
    }


    @Override
    public HealthState checkHealth() {
        if(motor == null || motor.isErrored()){
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }


    @Override
    public void mustangPeriodic() {
            //if cone then speed is inverted

        switch(status){
            case IDLE:
            if(gamepiece == GamePiece.CONE){
                motor.set(-RobotConstants.Arm.Claw.kIdleSpeed);
            }else{
                motor.set(RobotConstants.Arm.Claw.kIdleSpeed);
            }
                break;
            case INTAKING:
                if(gamepiece == GamePiece.CONE){
                    motor.set(-RobotConstants.Arm.Claw.kRollingSpeed);
                }else{
                    motor.set(RobotConstants.Arm.Claw.kRollingSpeed);
                }

                if(motor.getOutputCurrent()>RobotConstants.Arm.Claw.kCurrentMax){

                }
                //Variables:
                //RobotConstants.Arm.Claw.kCurrentSpikeIterations
                //currentSpikeCounter
                //isFull
                //setIdle()
                //Timer mTimer
                //mTimer.hasElapsed();
                //in start ejecting call mTimer.restart
                //mTimer.stop()

                //
              
                break;
            case EJECTING:
                break;
            default:
                motor.set(0);
        }

    }

    @Override
    public void debugSubsystem() {
       
    }

}
