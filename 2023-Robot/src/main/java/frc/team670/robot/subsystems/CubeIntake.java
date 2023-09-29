package frc.team670.robot.subsystems;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.LEDSubsystem.LEDColor;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;


public class CubeIntake extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, IDLE;
    }


    private SparkMAXLite motor;
    private CubeIntake.Status status=Status.IDLE;

    private final String currentKey = "CubeIntake motor current";
    private final String CubeIntakeStateKey = "CubeIntake state";
    protected Timer m_timer = new Timer();
    private int currentSpikeCounter = 0;
    private int ejectCounter = 0;
    private boolean isFull = false;
    private double ejectingSpeed =
            RobotConstants.CubeIntake.kEjectingSpeed;
    

    private LED led;
    private Deployer deployer;

    private static CubeIntake mInstance;

    public static synchronized CubeIntake getInstance() {
        mInstance = mInstance == null ? new CubeIntake() : mInstance;
        return mInstance;
    }

    public CubeIntake() {

        motor = SparkMAXFactory.buildSparkMAX(
                RobotConstants.CubeIntake.kMotorID,
                SparkMAXFactory.defaultConfig, Motor_Type.NEO);
        deployer=Deployer.getInstance();
        this.led = LED.getInstance();

        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
    }
    public Deployer getDeployer() {
        return deployer;
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
        this.startEjecting(RobotConstants.CubeIntake.kEjectingSpeed);
    }
    public void toggleDeployer(){

        deployer.toggleDeployer();
    }
 

    /**
     * Ejects the held item at the given speed
     * 
     * @param ejectingSpeed Should be <0. The more negative, the faster the CubeIntake will run backwards.
     */
    public void startEjecting(double ejectingSpeed) {
        this.ejectingSpeed = ejectingSpeed;
      
        // deployer.deploy(true);
        setStatus(Status.EJECTING);
        m_timer.restart();
    }

    public void startIntaking() {

        setStatus(Status.INTAKING);
    }

    /**
     * Sets the CubeIntake to an IDLE state Please note that "IDLE" does not mean "stopped"!
     */
    public void setIdle() {

        setStatus(Status.IDLE);
    }

    /**
     * Private method, only intended to be used by the public set() methods
     * 
     * @param status
     */
    private void setStatus(CubeIntake.Status status) {
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

            motor.set(RobotConstants.CubeIntake.kIdleSpeed);
            
                break;
            case INTAKING:
           
                motor.set(RobotConstants.CubeIntake.kRollingSpeed);
            
                
                if (motor.getOutputCurrent() > RobotConstants.CubeIntake.kCurrentMax) {
                    currentSpikeCounter++;
                    if (currentSpikeCounter > RobotConstants.CubeIntake.kCurrentSpikeIterations) {
                        isFull = true;
                        if (DriverStation.isTeleopEnabled()) {
                            led.solidhsv(LEDColor.GREEN);
                        }
                        
                       
                        // OI.getDriverController().rumble(0.5, 0.5);
                        // OI.getOperatorController().rumble(0.5, 0.5);
                        currentSpikeCounter = 0;
                        setIdle();
                        deployer.deploy(false);

                        Claw.getInstance().setIdle();
                        
                    }
                } else {
                    currentSpikeCounter = 0;
                }
                break;

            case EJECTING:
               
                motor.set(RobotConstants.CubeIntake.kEjectingSpeed);
                
                
                
                if (m_timer.hasElapsed(RobotConstants.Arm.Claw.kEjectTime)) {
                    m_timer.stop();
                    isFull = false;
                    setIdle();
                    Claw.getInstance().setIdle();
                    
                }
                
                break;
            default:
                motor.set(0);
        }
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber(currentKey, motor.getOutputCurrent());
        SmartDashboard.putString(CubeIntakeStateKey, status.toString());
        
    }



/**
 * Represents the deployer for the intake
 * 
 * @author armaangomes
 */
    public static class Deployer extends MustangSubsystemBase {
        public record Config(int kMotorID, int kSlot, MotorConfig.Motor_Type kMotorType,
        IdleMode kIdleMode,double  kGearRatio, int kContinuousCurrent, int kPeakCurrent){}
        
    

    private double time = 0;
    // private double errorCounter = 0;
    private boolean deployed=false;
    private SparkMAXLite mRotator;
    // private final String current = "Deployer current";
    private final Config kConfig;
    private static Deployer mInstance;
    protected Timer m_timer = new Timer();
    // constructor that inits motors and stuff

    public Deployer() {
        
        kConfig=RobotConstants.CubeIntake.Deployer.kConfig;
        mRotator= SparkMAXFactory.buildFactorySparkMAX(kConfig.kMotorID, kConfig.kMotorType);
        mRotator.setInverted(true);
        mRotator.setSmartCurrentLimit(kConfig.kPeakCurrent, kConfig.kContinuousCurrent);
    }

   



    @Override
    public HealthState checkHealth() {
        if (mRotator == null || mRotator.isErrored()) {
            return HealthState.RED;
        }
        return HealthState.GREEN;

       
    }









    @Override
    public void debugSubsystem() {
        // double relativePosition = super.mEncoder.getPosition();

        // SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
        // SmartDashboard.putNumber(positionRot, relativePosition);
        // SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
        // SmartDashboard.putNumber(setpointRot, mSetpoint);
        // SmartDashboard.putNumber(current, super.getRotator().getOutputCurrent());
        // SmartDashboard.putNumber("Deployer motor power: ", super.mRotator.get());
        SmartDashboard.putNumber("Deployer motor power: ", mRotator.get());
        SmartDashboard.putBoolean("Deployer deployeds", deployed);
    }

    @Override
    public void mustangPeriodic() {
      
      if(m_timer.hasElapsed(time)){
        m_timer.reset();
        m_timer.stop();
        mRotator.stopMotor();
        if(deployed){
            // mRotator.setSmartCurrentLimit(10);
            mRotator.set(RobotConstants.CubeIntake.Deployer.kDeployedIdleSpeed);
          }
      }
      
    }

        public static synchronized Deployer getInstance() {
            mInstance = mInstance == null ? new Deployer() : mInstance;
            return mInstance;
        }
 
        public void toggleDeployer() {
            
            if ( !isDeployed()) {
                deploy(true);
                
            } else {
                // SmartDashboard.putNumber("Deployed deployer",180);
                deploy(false);
            }
           
         
        }
        public void deploy(boolean deploy){
            if(deploy==true){
                time=RobotConstants.CubeIntake.Deployer.kTimeDown;
                m_timer.restart();
                mRotator.set(RobotConstants.CubeIntake.Deployer.kMotorSpeed);
                setRotatorMode(true);
                deployed=true;
            }else{

                time=RobotConstants.CubeIntake.Deployer.kTimeUp;
                m_timer.restart();
                mRotator.set(-RobotConstants.CubeIntake.Deployer.kMotorSpeed);
                setRotatorMode(false);
                deployed=false;
            }
        }

        /**
         * @param true to set flipout to coast mode, false to set it to brake mode
         */
        public void setRotatorMode(boolean coast) {
            if (coast) {
                mRotator.setIdleMode(IdleMode.kCoast);
            } else {
                mRotator.setIdleMode(IdleMode.kBrake);
            }
        }

        public boolean isDeployed() {
            return deployed;
        }

      




        
    }
}
