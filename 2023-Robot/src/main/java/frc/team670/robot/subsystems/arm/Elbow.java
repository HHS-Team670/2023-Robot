
package frc.team670.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystemIO;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.robot.constants.RobotConstants;

/**
 * Represents the Elbow joint. Uses only one motor
 * 
 * @author Armaan Aditi Kedar Gabriel Alexander Justin Sanatan Srinish
 */
public class Elbow extends Joint {
  

    // constructor that inits motors and stuff

    public Elbow() {
        super(new ElbowIO(), new JointIOInputsAutoLogged());
       
    }
    public void sendAngleToDashboard() {
        SmartDashboard.putNumber("Elbow Position Deg", getCurrentAngleInDegrees());
    }




    
    












    // @Override
    // public void debugSubsystem() {
    //     double relativePosition = super.mEncoder.getPosition();

    //     SmartDashboard.putNumber(positionDeg, getCurrentAngleInDegrees());
    //     SmartDashboard.putNumber(positionRot, relativePosition);
    //     SmartDashboard.putNumber(absEncoderPos, absEncoder.getAbsolutePosition());
    //     SmartDashboard.putNumber(setpointRot, mSetpoint);
    //     SmartDashboard.putNumber(current, super.getRotator().getOutputCurrent());
    //     SmartDashboard.putNumber("Elbow motor power: ", super.mRotator.get());
    // }

    




    public static class ElbowIO extends JointIO{

     

        public ElbowIO() {
            super(RobotConstants.Arm.Elbow.kConfig,RobotConstants.Arm.Elbow.kAbsoluteEncoderID);
            super.getRotator().setInverted(true);

        }

        @Override
        protected void setEncoderPositionFromAbsolute(JointIOInputs inputs) {
            double absEncoderPosition = absEncoder.getAbsolutePosition();
            double previousPositionRot = (inputs).mEncoderPositionUnadjusted;
    
            if (absEncoderPosition != 0.0) {
                double relativePosition = ((-1 * (absEncoderPosition
                        - (RobotConstants.Arm.Elbow.kAbsoluteEncoderVerticalOffset - 0.5)) + 2)
                        * RobotConstants.Arm.Elbow.kGearRatio) % RobotConstants.Arm.Elbow.kGearRatio;
    
                if (calculatedRelativePosition == 0.0
                        || Math.abs(360 * ((previousPositionRot - relativePosition)
                                / kConfig.kRotatorGearRatio())) < 10.0) {
                    clearSetpoint();
                    REVLibError error = mEncoder.setPosition(relativePosition);
                    SmartDashboard.putNumber("Elbow absEncoder position when reset",
                            absEncoderPosition);
                    SmartDashboard.putNumber("Elbow relEncoder position when reset", relativePosition);
                    SmartDashboard.putString("Elbow error", error.toString());
                    calculatedRelativePosition = relativePosition;
                }
            }
        }

      

    }
}
