
package frc.team670.robot.subsystems.arm;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystem;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.robot.constants.RobotConstants;

/**
 * Represents the wrist joint. Uses only one motor
 * 
 * @author Armaan Aditi Kedar Gabriel Alexander Justin
 */
public class Wrist extends Joint {

  

    private final String positionDeg = "Wrist position (deg)";
    private final String absEncoderPos = "Wrist abs encoder position";
    private final String positionRot = "Wrist position (rotations)";
    private final String setpointRot = "Wrist setpoint (rotations)";


    public Wrist() {
        super(new WristIO(), new JointIOInputsAutoLogged());



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
    public static class WristIO extends JointIO{

        public WristIO() {
            super(RobotConstants.Arm.Wrist.kConfig,RobotConstants.Arm.Wrist.kAbsoluteEncoderID);
            super.getRotator().setInverted(false);

        }

        @Override
        protected void setEncoderPositionFromAbsolute(JointIOInputs inputs) {
            double absEncoderPosition = absEncoder.getAbsolutePosition();
            double previousPositionRot = super.mEncoder.getPosition();
    
            if (absEncoderPosition != 0.0) {
                double relativePosition = ((1 * (absEncoderPosition
                        - (RobotConstants.Arm.Wrist.kAbsoluteEncoderVerticalOffset - 0.5)) + 1)
                        * RobotConstants.Arm.Wrist.kGearRatio) % RobotConstants.Arm.Wrist.kGearRatio;
    
                if (calculatedRelativePosition == 0.0
                        || Math.abs(360 * ((previousPositionRot - relativePosition)
                                / kConfig.kRotatorGearRatio())) < 20.0) {
                    clearSetpoint();
    
                    REVLibError error = mEncoder.setPosition(relativePosition);
                    SmartDashboard.putNumber("Wrist absEncoder position when reset",
                            absEncoderPosition);
                    SmartDashboard.putNumber("Wrist relEncoder position when reset", relativePosition);
                    SmartDashboard.putString("Wrist error", error.toString());
                    calculatedRelativePosition = relativePosition;
                }
    
            }
        }
        @Override 
        public HealthState checkHealth(){
            REVLibError rotatorError = super.mRotator.getLastError();

            if (rotatorError != null && rotatorError != REVLibError.kOk) {
                Logger.consoleError("Wrist error! Rotator Error is " + rotatorError.toString());
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


    }
}
