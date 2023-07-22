package frc.team670.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.arm.Joint.JointIO;

/**
 * Represents the shoulder joint. The shoulder uses a leader-follower SparkMax pair
 * 
 * @author Armaan, Kedar, Aditi, Justin, Alexander, Gabriel, Srinish, Sanatan
 */
public class Shoulder extends Joint {


    private final String positionDeg = "Shoulder position (deg)";
    // private final String absEncoderPos = "Shoulder abs encoder position";
    // private final String positionRot = "Shoulder position (rotations)";
    // private final String setpointRot = "Shoulder setpoint (rotations)";
    public static final double SHOULDER_ARBITRARY_FF = 0.5;


    public Shoulder() {
        super(new ShoulderIO(), new JointIOInputsAutoLogged());
    }

    // @Override
    // public void debugSubsystem() {
    //     double relativePosition = super.mEncoder.getPosition();
    //     SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
    //     SmartDashboard.putNumber(positionRot, relativePosition);
    //     SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
    //     SmartDashboard.putNumber(setpointRot, mSetpoint);

    // }

    

    

    public void sendAngleToDashboard() {
        SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
    }


    public static class ShoulderIO extends JointIO{
        private SparkMAXLite follower;
        public ShoulderIO() {
            super(RobotConstants.Arm.Shoulder.kConfig, RobotConstants.Arm.Shoulder.kAbsoluteEncoderID);
            super.getRotator().setInverted(true);
            follower = SparkMAXFactory
                    .setPermanentFollower(RobotConstants.Arm.Shoulder.kFollowerMotorID, mRotator, true);
            follower.setIdleMode(IdleMode.kBrake);
        }
        @Override
        public HealthState checkHealth() {
            REVLibError leaderRotatorError = super.mRotator.getLastError();
            REVLibError followerRotatorError = follower.getLastError();
    
            boolean leaderOK = (leaderRotatorError == REVLibError.kOk);
            boolean followerOK = (followerRotatorError == REVLibError.kOk);
    
            if (!leaderOK && !followerOK) {
                Logger.consoleError("Shoulder error! Leader error is " + leaderRotatorError.toString());
                Logger.consoleError(
                        "Shoulder error! Follower error is " + followerRotatorError.toString());
                return HealthState.RED;
            }
    
            if ((leaderOK && !followerOK) || (!leaderOK && followerOK) || !hasSetAbsolutePosition
                    || !relativePositionIsSet) {
                return HealthState.YELLOW;
            }
    
            return HealthState.GREEN;
    
        }

        @Override
        protected void setEncoderPositionFromAbsolute(JointIOInputs inputs) {
            // TODO Auto-generated method stub
            double absEncoderPosition = absEncoder.getAbsolutePosition();
        double previousPositionRot = super.mEncoder.getPosition();
        if (absEncoderPosition != 0.0) {

            double relativePosition = ((-1
                    * (absEncoderPosition
                            - (RobotConstants.Arm.Shoulder.kAbsoluteEncoderVerticalOffset - 0.5))
                    + 1) * RobotConstants.Arm.Shoulder.kGearRatio)
                    % RobotConstants.Arm.Shoulder.kGearRatio;

            if (calculatedRelativePosition == 0.0
                    || Math.abs(360 * ((previousPositionRot - relativePosition)
                            / kConfig.kRotatorGearRatio())) < 5.0) {
                clearSetpoint();
                REVLibError error = mEncoder.setPosition(relativePosition);
                SmartDashboard.putNumber("Shoulder absEncoder position when reset",
                        absEncoderPosition);
                SmartDashboard.putNumber("Shoulder relEncoder position when reset",
                        relativePosition);
                SmartDashboard.putString("Shoulder error", error.toString());
                calculatedRelativePosition = relativePosition;
            }

        }
        }
        
    }
}
