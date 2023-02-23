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

import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends MustangSubsystemBase {

    public enum Status {
        EJECTING, INTAKING, IDLE;
    }

    private SparkMAXLite leader, follower;

    private int count = 0;
    private boolean isFull = false;
    private int ticker = 0;
    private Claw.Status status;
    private Arm arm;
    private boolean returnToStowed = true;

    public Claw(Arm arm) {
        List<SparkMAXLite> motorControllers = SparkMAXFactory.buildSparkMAXPair(RobotMap.CLAW_LEADER_MOTOR,
                RobotMap.CLAW_FOLLOWER_MOTOR, true, SparkMAXFactory.defaultConfig, Motor_Type.NEO_550);
        leader = motorControllers.get(0);
        follower = motorControllers.get(1);
        status = Status.IDLE;
        leader.setIdleMode(IdleMode.kBrake);
        follower.setIdleMode(IdleMode.kBrake);
        this.arm = arm;
    }

    public void setArm(Arm arm) {
        this.arm = arm;
    }

    public Arm getArm() {
        return arm;
    }

    public boolean willReturnToStowed() {
        return returnToStowed;
    }

    public void setReturnToStowed(boolean returnToStowed) {
        this.returnToStowed = returnToStowed;
    }

    public void setStatus(Claw.Status status) {
        this.status = status;
    }

    private void stopAll() {
        leader.set(0);
    }

    public boolean isFull() {
        return isFull;

    }

    @Override
    // Checking for hardware breaks within the motors.
    public HealthState checkHealth() {
        if (leader == null || leader.isErrored() || follower == null || follower.isErrored()) {
            return HealthState.RED;
        }
        return HealthState.GREEN;
    }

    @Override
    public void mustangPeriodic() {
        debugSubsystem();

        switch (status) {
            case IDLE:
                leader.set(RobotConstants.CLAW_IDLE_SPEED);
                break;
            case INTAKING:
                if (isFull) {
                    setStatus(Status.IDLE);
                }
                leader.set(RobotConstants.CLAW_ROLLING_SPEED);
                break;

            case EJECTING:
                leader.set(RobotConstants.CLAW_EJECTING_SPEED);
                ticker++;
                // some arbitrary amount of time until ejection is finished
                // TODO: adjust length of time or find a better way to check for finishing
                // ejection
                if (ticker > 25) {
                    isFull = false;
                    ticker = 0;
                    if (returnToStowed) {
                        MustangScheduler.getInstance().schedule(new MoveToTarget(arm, this, ArmState.STOWED));
                    }
                }
                break;
            default:
                leader.set(0);
        }

        if (this.status == Status.INTAKING) {
            if (leader.getOutputCurrent() > RobotConstants.CLAW_CURRENT_MAX
                    || follower.getOutputCurrent() > RobotConstants.CLAW_CURRENT_MAX) {
                count++;
                if (count > 5) {
                    setStatus(Status.IDLE);
                    isFull = true;
                    ticker = 0;
                    if (returnToStowed) {
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
        SmartDashboard.putNumber("Claw leader current", leader.getOutputCurrent());
        SmartDashboard.putNumber("Claw follower current", leader.getOutputCurrent());
        SmartDashboard.putString("Claw state", status.toString());
    }

}