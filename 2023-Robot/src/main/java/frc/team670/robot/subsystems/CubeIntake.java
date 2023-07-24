package frc.team670.robot.subsystems;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.LEDSubsystem.LEDColor;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Claw.GamePiece;


public class CubeIntake extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, IDLE;
    }


    private SparkMAXLite motor;
    private CubeIntake.Status status;

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
               
                motor.set(ejectingSpeed);
                
                
                ejectCounter++;
                if (ejectCounter > RobotConstants.CubeIntake.kEjectIterations) {
                    ejectCounter = 0;
                    isFull = false;
                    
                    
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
 * @author lakshbhambhani
 */
    public static class Deployer extends SparkMaxRotatingSubsystem {

        private DutyCycleEncoder absEncoder;
        private final Config kConfig;
        private static Deployer mInstance;

        
        


        private boolean rotatorSetpointCancelled = false;

        private double angle; 


        public Deployer() {
            super(RobotConstants.CubeIntake.Deployer.kConfig);
            setName("Deployer");
            kConfig=RobotConstants.CubeIntake.Deployer.kConfig;
            // setLogFileHeader("abs-Encoder", "rel-encoder", "rel-encoder-vel", "angle", "isDeployed");
            absEncoder = new DutyCycleEncoder(RobotConstants.CubeIntake.kAbsoluteEncoderID);
            setEncoderPositionFromAbsolute();
            enableCoastMode();
        }

        public static synchronized Deployer getInstance() {
            mInstance = mInstance == null ? new Deployer() : mInstance;
            return mInstance;
        }
        public double getSpeed() {
            return mRotator.get();
        }

        public double getAbsoluteEncoderRotations() {
            double pos = absEncoder.get();
            while (pos > RobotConstants.CubeIntake.Deployer.kAbsoluteEncoderPostionAtFlipoutMax + 0.1) {
                pos -= 1;
            }
            while (pos < RobotConstants.CubeIntake.Deployer.kAbsoluteEncoderPostionAtFlipoutZero - 0.1) {
                pos += 1;
            }
            return pos;
        }

        /**
         * Sets the rotator encoder's reference position to the constant obtained from
         * the absolute encoder corresponding to that position.
         */
        public void setEncoderPositionFromAbsolute() {
            clearSetpoint();
            
            mEncoder.setPosition(
                    -1 * (getAbsoluteEncoderRotations() - RobotConstants.CubeIntake.Deployer.kAbsoluteEncoderPostionAtFlipoutZero)
                            * kConfig.kRotatorGearRatio());
            // Logger.consoleLog("Encoder position set: %s", rotator_encoder.getPosition());
        }

        /**
         * @return the position, in number of rotations of the flipout
         */
        public double getPosition() {
            return mEncoder.getPosition();
        }

        @Override
        public HealthState checkHealth() {

            if (mRotator.isErrored()) {
                return HealthState.RED;
            }
            return HealthState.GREEN;
        }

        /**
         * Converts an intake angle into rotations
         */
        private static int convertFlipoutDegreesToRotations(double degrees) {
            // If straight up is 0 and going forward is positive
            // percentage * half rotation
            return (int) ((degrees / 90) *RobotConstants.CubeIntake.Deployer.kMaxFlipOutRotations);
        }

        /**
         * Converts intake rotations into an angle
         */
        private static double convertFlipoutRotationsToDegrees(double rotations) {
            // If straight up is 0 and going forward is positive
            return ((90 * rotations) / RobotConstants.CubeIntake.Deployer.kMaxFlipOutRotations);
        }

        /**
         * Turns to a target angle the most efficient way.
         */
        @Override
        public boolean setSystemTargetAngleInDegrees(double angleDegrees) {
            angleDegrees = MathUtil.clamp(angleDegrees, 0, 90);
            double currentAngle = getCurrentAngleInDegrees();
            if (angleDegrees > currentAngle) {
                
                mController.setSmartMotionMaxAccel(RobotConstants.CubeIntake.Deployer.kMaxAccelDownwards, 0);
            } else {
                mController.setSmartMotionMaxAccel(RobotConstants.CubeIntake.Deployer.kMaxAccelUpwards, 0);
            }
            setSystemMotionTarget(convertFlipoutDegreesToRotations(angleDegrees));
            return true;
        }

        /**
         * @return the angle the flipout is currently turned to, between 0 and 90
         */
        @Override
        public double getCurrentAngleInDegrees() {
            return convertFlipoutRotationsToDegrees(getUnadjustedPosition());
        }

        @Override
        public void mustangPeriodic() {
            debugSubsystem();
            if (angle == 90 && getCurrentAngleInDegrees() > 70 && !rotatorSetpointCancelled) {
                clearSetpoint();
                rotatorSetpointCancelled = true;
            }
        }

        public boolean hasReachedTargetPosition() {
            boolean hasReachedTarget = (mRotator.get() == 0
                    || MathUtils.doublesEqual(mEncoder.getPosition(), mSetpoint, kConfig.kAllowedErrorDegrees()));
            return hasReachedTarget;
        }

        public boolean deploy(boolean deploy) {
            angle = 0;
            if (deploy) {
                angle = 90;
                setRotatorMode(true);
                rotatorSetpointCancelled = false;
            } else {
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
        public boolean getTimeout() {
            return false;
        }

        @Override
        public void moveByPercentOutput(double output) {

        }

        @Override
        public void debugSubsystem() {
            SmartDashboard.putNumber("abs-Encoder", absEncoder.get());
            SmartDashboard.putNumber("rel-Encoder", this.mEncoder.getPosition());
            SmartDashboard.putNumber("rel-Encoder-vel", this.mEncoder.getVelocity());
            SmartDashboard.putNumber("angle", getCurrentAngleInDegrees());
            SmartDashboard.putBoolean("isDeployed", isDeployed());

            // writeToLogFile(absEncoder.get(), this.rotator_encoder.getPosition(), this.rotator_encoder.getVelocity(),
                    // getCurrentAngleInDegrees(), isDeployed());
        }
    }
}
