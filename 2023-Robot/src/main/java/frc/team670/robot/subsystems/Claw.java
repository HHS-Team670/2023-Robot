package frc.team670.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBaseIO;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Claw.ClawIO.ClawIOInputs;

public class Claw extends MustangSubsystemBase {
    private ClawIO io;
    private ClawIOInputs inputs;
    private Status status = Status.IDLE;
    private GamePiece gamePiece = GamePiece.CONE;
    private static Claw mInstance;

    public static synchronized Claw getInstance(){
        if(mInstance == null){
            mInstance = new Claw(new ClawIO(), new ClawIOInputsAutoLogged());
        }
        return mInstance;
    }
    public enum Status {
        EJECT, INTAKE, IDLE
    }

    public enum GamePiece {
        CONE, CUBE
    }

    public Claw(ClawIO io, LoggableInputs inputs) {

        super(io, inputs);
        this.io = io;
        this.inputs = (ClawIOInputs) inputs;
    }

    public boolean isFull() {
        return io.isFull();
    }

    public void startEjecting() {
        setStatus(Status.EJECT);
    }

    public void startIntaking() {
        setStatus(Status.INTAKE);
    }

    public void setIdle() {
        setStatus(Status.IDLE);
    }

    public void setGamePiece(GamePiece gampiece) {
        this.gamePiece = gampiece;
    }

    public void setStatus(Status status) {
        this.status = status;
    }

    @Override
    public void mustangPeriodic() {
        io.setMotorFromStatus(inputs, status, gamePiece);
    }

    public static class ClawIO extends MustangSubsystemBaseIO {
        private SparkMAXLite motor;
        private boolean isFull;
        private int currentSpikeCounter;
        private int ejectCounter;

        public ClawIO() {
            motor = SparkMAXFactory.buildSparkMAX(RobotConstants.Arm.Claw.kMotorID,
                    SparkMAXFactory.defaultConfig, Motor_Type.NEO);
            motor.setInverted(true);
            motor.setIdleMode(IdleMode.kBrake);
        }

        public boolean isFull() {
            return isFull;
        }

        @AutoLog
        public static class ClawIOInputs {
            public double motorCurrent = 0;

        }

        @Override
        public void updateInputs(LoggableInputs inputs) {
            ClawIOInputsAutoLogged input = (ClawIOInputsAutoLogged) inputs;
            input.motorCurrent = motor.getOutputCurrent();
        }

        @Override
        protected HealthState checkHealth() {
            return HealthState.GREEN;
        }

        public Status setMotorFromStatus(ClawIOInputs input, Status status, GamePiece gamepiece) {
            if (status == Status.INTAKE) {
                if (gamepiece == GamePiece.CONE) {
                    motor.set(-RobotConstants.Arm.Claw.kRollingSpeed);
                } else
                    motor.set(RobotConstants.Arm.Claw.kRollingSpeed);
                if (input.motorCurrent > RobotConstants.Arm.Claw.kCurrentMax) {
                    currentSpikeCounter++;
                    if (currentSpikeCounter > RobotConstants.Arm.Claw.kCurrentSpikeIterations) {
                        isFull = true;
                        currentSpikeCounter = 0;
                        return Status.IDLE;
                    }
                } else {
                    currentSpikeCounter = 0;

                }

            } else if (status == Status.EJECT) {
                if (gamepiece == GamePiece.CONE) {
                    motor.set(-RobotConstants.Arm.Claw.kEjectingSpeed);
                } else
                    motor.set(RobotConstants.Arm.Claw.kEjectingSpeed);
                ejectCounter++;
                if (ejectCounter > RobotConstants.Arm.Claw.kEjectIterations) {
                    ejectCounter = 0;
                    isFull = false;
                    return Status.IDLE;
                }
            } else if (status == Status.IDLE) {
                if (gamepiece == GamePiece.CONE) {
                    motor.set(-RobotConstants.Arm.Claw.kIdleSpeed);
                } else
                    motor.set(RobotConstants.Arm.Claw.kIdleSpeed);
            }
            return status;
        }

        @Override
        public void debugOutputs() {
            // TODO Auto-generated method stub

        }
    }
}