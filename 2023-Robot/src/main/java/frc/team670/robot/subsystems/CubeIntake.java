package frc.team670.robot.subsystems;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
 

    /**
     * Ejects the held item at the given speed
     * 
     * @param ejectingSpeed Should be <0. The more negative, the faster the CubeIntake will run backwards.
     */
    public void startEjecting(double ejectingSpeed) {
        this.ejectingSpeed = ejectingSpeed;
        if (this.status != Status.EJECTING) {
            this.isFull = true;
        }
        deployer.deploy(true);
        setStatus(Status.EJECTING);
    }

    public void startIntaking() {
        if (this.status != Status.INTAKING) {
            this.isFull = false;
        }
        deployer.deploy(true);
        setStatus(Status.INTAKING);
    }

    /**
     * Sets the CubeIntake to an IDLE state Please note that "IDLE" does not mean "stopped"!
     */
    public void setIdle() {
        deployer.deploy(false);
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
                        Claw.getInstance().setIdle();
                        
                    }
                } else {
                    currentSpikeCounter = 0;
                }
                break;

            case EJECTING:
               
                motor.set(RobotConstants.CubeIntake.kEjectingSpeed);
                
                
                ejectCounter++;
                if (ejectCounter > RobotConstants.CubeIntake.kEjectIterations) {
                    ejectCounter = 0;
                    isFull = false;
                    
                    
                }
                setIdle();
                
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
 * @author lakshbhambhani
 */
    public static class Deployer extends MustangSubsystemBase {
        public record Config(int kMotorID, int kSlot, MotorConfig.Motor_Type kMotorType,
        IdleMode kIdleMode,double  kGearRatio, int kContinuousCurrent,int kPeakCurrent){}
        
    

    private int counter = 0;
    private double errorCounter = 0;
    private boolean deployed=false;
`   private SparkMAXLite mRotator;
    private final String current = "Deployer current";
    private final Config kConfig;
    private static Deployer mInstance;
    // constructor that inits motors and stuff

    public Deployer() {
        
        kConfig=RobotConstants.CubeIntake.Deployer.kConfig;
        mRotator= SparkMAXFactory.buildFactorySparkMAX(kConfig.kMotorID, kConfig.kMotorType);
        mRotator.setInverted(true);
    }

   



    @Override
    public HealthState checkHealth() {
        // REVLibError rotatorError = super.mRotator.getLastError();

        // if (rotatorError != null && rotatorError != REVLibError.kOk) {
        //     Logger.consoleError("Deployer error! Rotator error is " + rotatorError.toString());
        //     errorCounter++;
        // } else {
        //     errorCounter = 0;
        // }

        // if (errorCounter >= 20) {
        //     return HealthState.RED;
        // }

        // if (!hasSetAbsolutePosition || !relativePositionIsSet) {
        //     return HealthState.YELLOW;
        // }

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
    }

    @Override
    public void mustangPeriodic() {
      
    }


        // private DutyCycleEncoder absEncoder;
      

        
        






        public static synchronized Deployer getInstance() {
            mInstance = mInstance == null ? new Deployer() : mInstance;
            return mInstance;
        }
 

        /**
         * @return the position, in number of rotations of the flipout
         */
        public double getPosition() {
            return mEncoder.getPosition();
        }



        public boolean deploy(boolean deploy) {
            angle = 180;
            if (deploy) {
                SmartDashboard.putNumber("Deployed deployer",90);
                angle = 90;
                setRotatorMode(true);
                rotatorSetpointCancelled = false;
                setSystemTargetAngleInDegrees(angle);
            } else {
                SmartDashboard.putNumber("Deployed deployer",180);
                setRotatorMode(false);
            }
            setSystemTargetAngleInDegrees(angle);
            return hasReachedTargetPosition();
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
            return (angle == 90);
        }

      

        @Override
        public void moveByPercentOutput(double output) {

        }

        // @Override
        // public void debugSubsystem() {
        //     SmartDashboard.putNumber("abs-Encoder", absEncoder.get());
        //     SmartDashboard.putNumber("rel-Encoder", this.mEncoder.getPosition());
        //     SmartDashboard.putNumber("rel-Encoder-vel", this.mEncoder.getVelocity());
        //     SmartDashboard.putNumber("angle", getCurrentAngleInDegrees());
        //     SmartDashboard.putBoolean("isDeployed", isDeployed());

        //     // writeToLogFile(absEncoder.get(), this.rotator_encoder.getPosition(), this.rotator_encoder.getVelocity(),
        //             // getCurrentAngleInDegrees(), isDeployed());
        // }
    }
}
