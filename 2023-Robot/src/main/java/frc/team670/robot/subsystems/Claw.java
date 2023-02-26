package frc.team670.robot.subsystems;

import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.commands.arm.MoveToTarget;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.arm.Arm;
import frc.team670.robot.subsystems.arm.ArmState;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, IDLE;
    }

    private SparkMAXLite motor;
    private int count = 0;
    private int ejectingCount = 0;
    private Claw.Status status;
    private Arm arm;
    private boolean returnToStowedAfterIntake = true;
    private boolean returnToStowedAfterEject = true;
    private double ejectingSpeed = RobotConstants.CLAW_EJECTING_SPEED;

    public Claw(Arm arm) {
        this.arm = arm;
        motor = SparkMAXFactory.buildSparkMAX(RobotMap.CLAW_MOTOR, SparkMAXFactory.defaultConfig, Motor_Type.NEO);
        status = Status.IDLE;
        motor.setIdleMode(IdleMode.kBrake);
    }

    public Arm getArm() {
        return arm;
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

    public void startIntaking(boolean returnToStowed) {
        this.returnToStowedAfterIntake = returnToStowed;
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
                motor.set(RobotConstants.CLAW_ROLLING_SPEED);
                break;

            case EJECTING:
                motor.set(ejectingSpeed);
                ejectingCount++;
                // some arbitrary amount of time until ejection is finished
                // TODO: adjust length of time or find a better way to check for finishing
                // ejection
                if (ejectingCount > 25) {
                    ejectingCount = 0;
                    if (returnToStowedAfterEject) { //Always true for now, but can change
                        MustangScheduler.getInstance().schedule(new MoveToTarget(arm, this, ArmState.STOWED));
                    }
                }
                break;
            default:
                motor.set(0);
        }

        // If the current has spiked for more than 1/10th of a second, then set the status to idle
        if(this.status == Status.INTAKING) {
            if(motor.getOutputCurrent() > RobotConstants.CLAW_CURRENT_MAX) {
                count++;
                if(count > 5) {
                    setIdle();
                    if (returnToStowedAfterIntake) {
                        MustangScheduler.getInstance().schedule(new MoveToTarget(arm, this, ArmState.STOWED));
                    }
                }
            } else {
                count = 0;
            }

        }
    }

    @Override
    public void debugSubsystem() {
        SmartDashboard.putNumber("Claw motor current", motor.getOutputCurrent());
        SmartDashboard.putString("Claw state", status.toString());
    }

}