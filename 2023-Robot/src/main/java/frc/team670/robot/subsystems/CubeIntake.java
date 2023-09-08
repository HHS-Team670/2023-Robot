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
    private boolean hasSetAbsolutePosition = false;
    private int counter = 0;
    private double previousReading = 0.0;
    private double calculatedRelativePosition = 0.0;
    private boolean relativePositionIsSet = false;
    private double errorCounter = 0;

    private final String positionDeg = "Deployer position (deg)";
    private final String absEncoderPos = "Deployer abs encoder position";
    private final String positionRot = "Deployer position (rotations)";
    private final String setpointRot = "Deployer setpoint (rotations)";
    private final String current = "Deployer current";

    // constructor that inits motors and stuff

    public Deployer() {
        super(RobotConstants.CubeIntake.Deployer.kConfig);
        kConfig=RobotConstants.CubeIntake.Deployer.kConfig;
        absEncoder = new DutyCycleEncoder(RobotConstants.CubeIntake.Deployer.kAbsoluteEncoderID);
        super.getRotator().setInverted(true);
    }

    /**
     * PRIVATE method to set position from absolute. Do not use directly. Instead, use
     * resetPositionFromAbsolute()
     */
    private void setEncoderPositionFromAbsolute() {
        double absEncoderPosition = absEncoder.getAbsolutePosition();
        double previousPositionRot = super.mEncoder.getPosition();

        if (absEncoderPosition != 0.0) {
            double relativePosition = ((-1 * (absEncoderPosition
                    - (RobotConstants.CubeIntake.Deployer.kAbsoluteEncoderVerticalOffset - 0.5)) + 2)
                    * RobotConstants.CubeIntake.Deployer.kGearRatio) % RobotConstants.CubeIntake.Deployer.kGearRatio;

            if (calculatedRelativePosition == 0.0
                    || Math.abs(360 * ((previousPositionRot - relativePosition)
                            / kConfig.kRotatorGearRatio())) < 10.0) {
                clearSetpoint();
                REVLibError error = mEncoder.setPosition(relativePosition);
                SmartDashboard.putNumber("Deployer absEncoder position when reset",
                        absEncoderPosition);
                SmartDashboard.putNumber("Deployer relEncoder position when reset", relativePosition);
                SmartDashboard.putString("Deployer error", error.toString());
                calculatedRelativePosition = relativePosition;
            }
        }

    }

    @Override
    public boolean getTimeout() {
        return false;
    }

    @Override
    public HealthState checkHealth() {
        REVLibError rotatorError = super.mRotator.getLastError();

        if (rotatorError != null && rotatorError != REVLibError.kOk) {
            Logger.consoleError("Deployer error! Rotator error is " + rotatorError.toString());
            errorCounter++;
        } else {
            errorCounter = 0;
        }

        if (errorCounter >= 20) {
            return HealthState.RED;
        }

        if (!hasSetAbsolutePosition || !relativePositionIsSet) {
            return HealthState.YELLOW;
        }

        return HealthState.GREEN;
    }

    /**
     * Returns whether or not the relative position has been properly set from the absEncoder. When
     * resetPositionFromAbsolute() gets called, this will temporarily be false.
     */
    public boolean isRelativePositionSet() {
        return relativePositionIsSet;
    }

    /**
     * Public method to reset the position from the absolute position.
     */
    public void resetPositionFromAbsolute() {
        setEncoderPositionFromAbsolute();
    }

    // @Override
    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(mEncoder.getPosition(), mSetpoint,
        RobotConstants.CubeIntake.Deployer.kAllowedErrorRotations));
    }

    @Override
    public boolean setSystemTargetAngleInDegrees(double targetAngle) {
     
        return super.setSystemTargetAngleInDegrees(targetAngle);
    }

    @Override
    public void debugSubsystem() {
        double relativePosition = super.mEncoder.getPosition();

        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
        SmartDashboard.putNumber(positionRot, relativePosition);
        SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(setpointRot, mSetpoint);
        SmartDashboard.putNumber(current, super.getRotator().getOutputCurrent());
        SmartDashboard.putNumber("Deployer motor power: ", super.mRotator.get());
    }

    @Override
    public void mustangPeriodic() {
        if (!hasSetAbsolutePosition) { // before it's set an absolute position...
            double position = absEncoder.getAbsolutePosition();
            if (Math.abs(previousReading - position) < 0.02 && position != 0.0) { // If the current
                                                                                  // reading is
                                                                                  // PRECISELY
                                                                                  // 0, then it's
                                                                                  // not valid.
                counter++; // increases the counter if the current reading is close enough to the
                           // last
                           // reading.
                           // We do this because when the absEncoder gets initialized, its reading
                           // fluctuates drastically at the start.
            } else {
                counter = 0;
                previousReading = position;
            }
            if (counter > 100) { // Once it's maintained a constant value for long enough...
                setEncoderPositionFromAbsolute();
                hasSetAbsolutePosition = true;
            }
        } else if (!relativePositionIsSet) {
            double position = super.mEncoder.getPosition();
            Logger.consoleLog("Deployer relative position = " + position
                    + ", calculatedRelativePosition = " + calculatedRelativePosition);
            if (Math.abs(position - calculatedRelativePosition) < 0.5) {
                relativePositionIsSet = true;
            } else {
                super.mEncoder.setPosition(calculatedRelativePosition);
            }
            Logger.consoleLog("Deployer relativePositionIsSet = " + this.relativePositionIsSet);
        }
        if (angle == 90 && super.getCurrentAngleInDegrees() > 80 && !rotatorSetpointCancelled) {
            clearSetpoint();
            rotatorSetpointCancelled = true;
        }
    }

    public void sendAngleToDashboard() {
        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
    }
        // private DutyCycleEncoder absEncoder;
        private final Config kConfig;
        private static Deployer mInstance;

        
        


        private boolean rotatorSetpointCancelled = false;

        private double angle; 


        // public Deployer() {
        //     super(RobotConstants.CubeIntake.Deployer.kConfig);
        //     setName("Deployer");
        //     kConfig=RobotConstants.CubeIntake.Deployer.kConfig;
        //     // setLogFileHeader("abs-Encoder", "rel-encoder", "rel-encoder-vel", "angle", "isDeployed");
        //     absEncoder = new DutyCycleEncoder(RobotConstants.CubeIntake.kAbsoluteEncoderID);
        //     setEncoderPositionFromAbsolute();
        //     enableCoastMode();
        // }

        public static synchronized Deployer getInstance() {
            mInstance = mInstance == null ? new Deployer() : mInstance;
            return mInstance;
        }
        // public double getSpeed() {
        //     return mRotator.get();
        // }

        // public double getAbsoluteEncoderRotations() {
        //     double pos = absEncoder.get();
        //     while (pos > RobotConstants.CubeIntake.Deployer.kAbsoluteEncoderPostionAtFlipoutMax + 0.1) {
        //         pos -= 1;
        //     }
        //     while (pos < RobotConstants.CubeIntake.Deployer.kAbsoluteEncoderPostionAtFlipoutZero - 0.1) {
        //         pos += 1;
        //     }
        //     return pos;
        // }

        /**
         * Sets the rotator encoder's reference position to the constant obtained from
         * the absolute encoder corresponding to that position.
         */
        // public void setEncoderPositionFromAbsolute() {
        //     clearSetpoint();
            
        //     mEncoder.setPosition(
        //             -1 * (getAbsoluteEncoderRotations() - RobotConstants.CubeIntake.Deployer.kAbsoluteEncoderPostionAtFlipoutZero)
        //                     * kConfig.kRotatorGearRatio());
        //     // Logger.consoleLog("Encoder position set: %s", rotator_encoder.getPosition());
        // }

        /**
         * @return the position, in number of rotations of the flipout
         */
        public double getPosition() {
            return mEncoder.getPosition();
        }

        // @Override
        // public HealthState checkHealth() {

        //     if (mRotator.isErrored()) {
        //         return HealthState.RED;
        //     }
        //     return HealthState.GREEN;
        // }

        // /**
        //  * Converts an intake angle into rotations
        //  */
        // private static int convertFlipoutDegreesToRotations(double degrees) {
        //     // If straight up is 0 and going forward is positive
        //     // percentage * half rotation
        //     return (int) ((degrees / 90) *RobotConstants.CubeIntake.Deployer.kMaxFlipOutRotations);
        // }

        // /**
        //  * Converts intake rotations into an angle
        //  */
        // private static double convertFlipoutRotationsToDegrees(double rotations) {
        //     // If straight up is 0 and going forward is positive
        //     return ((90 * rotations) / RobotConstants.CubeIntake.Deployer.kMaxFlipOutRotations);
        // }

        // /**
        //  * Turns to a target angle the most efficient way.
        //  */
        // @Override
        // public boolean setSystemTargetAngleInDegrees(double angleDegrees) {
        //     angleDegrees = MathUtil.clamp(angleDegrees, 0, 90);
        //     double currentAngle = getCurrentAngleInDegrees();
        //     if (angleDegrees > currentAngle) {
                
        //         mController.setSmartMotionMaxAccel(RobotConstants.CubeIntake.Deployer.kMaxAccelDownwards, 0);
        //     } else {
        //         mController.setSmartMotionMaxAccel(RobotConstants.CubeIntake.Deployer.kMaxAccelUpwards, 0);
        //     }
        //     setSystemMotionTarget(convertFlipoutDegreesToRotations(angleDegrees));
        //     return true;
        // }

        // /**
        //  * @return the angle the flipout is currently turned to, between 0 and 90
        //  */
        // @Override
        // public double getCurrentAngleInDegrees() {
        //     return convertFlipoutRotationsToDegrees(getUnadjustedPosition());
        // }

        // @Override
        // public void mustangPeriodic() {
        //     debugSubsystem();
        //     SmartDashboard.putNumber("Deployer ANgle", getCurrentAngleInDegrees());
        //     if (angle == 90 && getCurrentAngleInDegrees() > 70 && !rotatorSetpointCancelled) {
        //         clearSetpoint();
        //         rotatorSetpointCancelled = true;
        //     }
        //     SmartDashboard.putNumber("Dployer absolute encoder position ", absEncoder.getAbsolutePosition());
        // }


        public boolean deploy(boolean deploy) {
            angle = 180;
            if (deploy) {
                SmartDashboard.putBoolean("Deployed deployer",true);
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
